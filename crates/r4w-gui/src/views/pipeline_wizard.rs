//! Pipeline-Based Waveform Builder
//!
//! A visual signal processing pipeline builder for creating waveform specifications.
//! Supports multiple blocks, parallel branches, and flexible routing.
//!
//! Features:
//! - 40+ signal processing blocks in 10 categories
//! - Visual canvas with grid, zoom, and pan
//! - Bezier curve connections between blocks
//! - Preset pipeline templates (BPSK TX, QPSK TX, LoRa TX, OFDM TX, etc.)
//! - Interactive connection creation (click output → click input)
//! - Block parameter editing for all block types
//! - YAML export for pipeline specifications
//! - Pipeline validation (unconnected inputs, cycles detection)

use egui::{Ui, RichText, Color32, Pos2, Vec2, Rect, Stroke, epaint::PathShape};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use super::block_metadata::{self, get_block_metadata};

/// Unique identifier for a pipeline block
pub type BlockId = u32;

/// Preset pipeline templates
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PipelinePreset {
    BpskTx,
    QpskTx,
    Qam16Tx,
    LoRaTx,
    OfdmTx,
    FskTx,
    DsssTx,
    DmrTx,
    BpskTxRx,
    QpskTxRx,
    OfdmTxRx,
    ParallelIqDemo,
}

impl PipelinePreset {
    pub fn name(&self) -> &'static str {
        match self {
            Self::BpskTx => "BPSK Transmitter",
            Self::QpskTx => "QPSK Transmitter",
            Self::Qam16Tx => "16-QAM Transmitter",
            Self::LoRaTx => "LoRa Transmitter",
            Self::OfdmTx => "OFDM Transmitter",
            Self::FskTx => "FSK Transmitter",
            Self::DsssTx => "DSSS Transmitter",
            Self::DmrTx => "DMR/4FSK Transmitter",
            Self::BpskTxRx => "BPSK TX → Channel → RX",
            Self::QpskTxRx => "QPSK TX → Channel → RX",
            Self::OfdmTxRx => "OFDM TX → Channel → RX",
            Self::ParallelIqDemo => "Parallel I/Q Demo",
        }
    }

    pub fn description(&self) -> &'static str {
        match self {
            Self::BpskTx => "Simple BPSK transmitter with pulse shaping",
            Self::QpskTx => "QPSK transmitter with RRC pulse shaping",
            Self::Qam16Tx => "16-QAM transmitter with coding and shaping",
            Self::LoRaTx => "LoRa CSS modulator with whitening",
            Self::OfdmTx => "OFDM transmitter with QPSK subcarriers",
            Self::FskTx => "2-FSK transmitter with filtering",
            Self::DsssTx => "Direct Sequence Spread Spectrum transmitter",
            Self::DmrTx => "DMR/P25 4-FSK transmitter with framing",
            Self::BpskTxRx => "Complete BPSK system with channel and recovery",
            Self::QpskTxRx => "Complete QPSK system with channel and recovery",
            Self::OfdmTxRx => "Complete OFDM system with fading channel",
            Self::ParallelIqDemo => "Demonstrates I/Q split and merge",
        }
    }

    pub fn all() -> &'static [PipelinePreset] {
        &[
            Self::BpskTx,
            Self::QpskTx,
            Self::Qam16Tx,
            Self::LoRaTx,
            Self::OfdmTx,
            Self::FskTx,
            Self::DsssTx,
            Self::DmrTx,
            Self::BpskTxRx,
            Self::QpskTxRx,
            Self::OfdmTxRx,
            Self::ParallelIqDemo,
        ]
    }
}

/// Validation result for pipeline
#[derive(Clone, Debug, Default)]
pub struct ValidationResult {
    pub is_valid: bool,
    pub warnings: Vec<String>,
    pub errors: Vec<String>,
}

/// Block category for organization
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockCategory {
    Source,
    Coding,
    Mapping,
    Modulation,
    Filtering,
    RateConversion,
    Synchronization,
    Impairments,
    Recovery,
    Output,
}

impl BlockCategory {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Source => "Source",
            Self::Coding => "Coding",
            Self::Mapping => "Mapping",
            Self::Modulation => "Modulation",
            Self::Filtering => "Filtering",
            Self::RateConversion => "Rate Conversion",
            Self::Synchronization => "Synchronization",
            Self::Impairments => "Impairments",
            Self::Recovery => "Recovery",
            Self::Output => "Output",
        }
    }

    pub fn color(&self) -> Color32 {
        match self {
            Self::Source => Color32::from_rgb(100, 200, 100),      // Green
            Self::Coding => Color32::from_rgb(200, 150, 100),      // Orange
            Self::Mapping => Color32::from_rgb(150, 150, 200),     // Purple
            Self::Modulation => Color32::from_rgb(100, 150, 200),  // Blue
            Self::Filtering => Color32::from_rgb(200, 100, 100),   // Red
            Self::RateConversion => Color32::from_rgb(200, 200, 100), // Yellow
            Self::Synchronization => Color32::from_rgb(100, 200, 200), // Cyan
            Self::Impairments => Color32::from_rgb(150, 100, 150), // Magenta
            Self::Recovery => Color32::from_rgb(150, 200, 150),    // Light green
            Self::Output => Color32::from_rgb(180, 180, 180),      // Gray
        }
    }

    pub fn all() -> &'static [BlockCategory] {
        &[
            Self::Source,
            Self::Coding,
            Self::Mapping,
            Self::Modulation,
            Self::Filtering,
            Self::RateConversion,
            Self::Synchronization,
            Self::Impairments,
            Self::Recovery,
            Self::Output,
        ]
    }
}

/// Type of processing block
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum BlockType {
    // Source blocks
    BitSource { pattern: String },
    SymbolSource { alphabet_size: u32 },
    FileSource { path: String },

    // Coding blocks
    Scrambler { polynomial: u32, seed: u32 },
    FecEncoder { code_type: FecType, rate: String },
    Interleaver { rows: u32, cols: u32 },
    CrcGenerator { crc_type: CrcType },

    // Mapping blocks
    GrayMapper { bits_per_symbol: u32 },
    ConstellationMapper { constellation: ConstellationType },
    DifferentialEncoder,

    // Modulation blocks
    PskModulator { order: u32 },
    QamModulator { order: u32 },
    FskModulator { deviation_hz: f32, order: u32 },
    OfdmModulator { fft_size: u32, cp_len: u32, data_carriers: u32 },
    DsssSpread { chips_per_symbol: u32, code_type: String },
    FhssHop { num_channels: u32, hop_rate: f32 },
    CssModulator { sf: u8, bw_hz: u32 },

    // Filtering blocks
    FirFilter { filter_type: FilterType, cutoff_hz: f32, num_taps: u32 },
    IirFilter { design: IirDesign, cutoff_hz: f32, order: u32 },
    PulseShaper { shape: PulseShape, rolloff: f32, span: u32 },
    MatchedFilter,

    // Rate conversion blocks
    Upsampler { factor: u32 },
    Downsampler { factor: u32 },
    RationalResampler { up: u32, down: u32 },
    PolyphaseResampler { up: u32, down: u32, taps_per_phase: u32 },

    // Synchronization blocks
    PreambleInsert { pattern: String, length: u32 },
    SyncWordInsert { word: String },
    FrameBuilder { header_bits: u32, payload_bits: u32 },
    TdmaFramer { slots: u32, slot_ms: f32 },

    // Impairments (for simulation)
    AwgnChannel { snr_db: f32 },
    FadingChannel { model: FadingModel, doppler_hz: f32 },
    FrequencyOffset { offset_hz: f32 },
    IqImbalance { gain_db: f32, phase_deg: f32 },

    // Demodulation blocks
    PskDemodulator { order: u32 },
    QamDemodulator { order: u32 },
    FskDemodulator { order: u32 },

    // Recovery blocks
    Agc { mode: AgcMode, target_db: f32 },
    TimingRecovery { algorithm: TimingAlgo, loop_bw: f32 },
    CarrierRecovery { algorithm: CarrierAlgo, loop_bw: f32 },
    Equalizer { eq_type: EqualizerType, taps: u32, mu: f32 },

    // Output blocks
    IqOutput,
    BitOutput,
    FileOutput { path: String, format: OutputFormat },

    // Control flow
    Split { num_outputs: u32 },
    Merge { num_inputs: u32 },
    IqSplit,
    IqMerge,
}

impl BlockType {
    pub fn name(&self) -> &'static str {
        match self {
            Self::BitSource { .. } => "Bit Source",
            Self::SymbolSource { .. } => "Symbol Source",
            Self::FileSource { .. } => "File Source",
            Self::Scrambler { .. } => "Scrambler",
            Self::FecEncoder { .. } => "FEC Encoder",
            Self::Interleaver { .. } => "Interleaver",
            Self::CrcGenerator { .. } => "CRC Generator",
            Self::GrayMapper { .. } => "Gray Mapper",
            Self::ConstellationMapper { .. } => "Constellation Mapper",
            Self::DifferentialEncoder => "Differential Encoder",
            Self::PskModulator { .. } => "PSK Modulator",
            Self::QamModulator { .. } => "QAM Modulator",
            Self::FskModulator { .. } => "FSK Modulator",
            Self::OfdmModulator { .. } => "OFDM Modulator",
            Self::DsssSpread { .. } => "DSSS Spreader",
            Self::FhssHop { .. } => "FHSS Hopper",
            Self::CssModulator { .. } => "CSS Modulator",
            Self::PskDemodulator { .. } => "PSK Demodulator",
            Self::QamDemodulator { .. } => "QAM Demodulator",
            Self::FskDemodulator { .. } => "FSK Demodulator",
            Self::FirFilter { .. } => "FIR Filter",
            Self::IirFilter { .. } => "IIR Filter",
            Self::PulseShaper { .. } => "Pulse Shaper",
            Self::MatchedFilter => "Matched Filter",
            Self::Upsampler { .. } => "Upsampler",
            Self::Downsampler { .. } => "Downsampler",
            Self::RationalResampler { .. } => "Rational Resampler",
            Self::PolyphaseResampler { .. } => "Polyphase Resampler",
            Self::PreambleInsert { .. } => "Preamble Insert",
            Self::SyncWordInsert { .. } => "Sync Word Insert",
            Self::FrameBuilder { .. } => "Frame Builder",
            Self::TdmaFramer { .. } => "TDMA Framer",
            Self::AwgnChannel { .. } => "AWGN Channel",
            Self::FadingChannel { .. } => "Fading Channel",
            Self::FrequencyOffset { .. } => "Frequency Offset",
            Self::IqImbalance { .. } => "IQ Imbalance",
            Self::Agc { .. } => "AGC",
            Self::TimingRecovery { .. } => "Timing Recovery",
            Self::CarrierRecovery { .. } => "Carrier Recovery",
            Self::Equalizer { .. } => "Equalizer",
            Self::IqOutput => "IQ Output",
            Self::BitOutput => "Bit Output",
            Self::FileOutput { .. } => "File Output",
            Self::Split { .. } => "Split",
            Self::Merge { .. } => "Merge",
            Self::IqSplit => "I/Q Split",
            Self::IqMerge => "I/Q Merge",
        }
    }

    pub fn category(&self) -> BlockCategory {
        match self {
            Self::BitSource { .. } | Self::SymbolSource { .. } | Self::FileSource { .. } => BlockCategory::Source,
            Self::Scrambler { .. } | Self::FecEncoder { .. } | Self::Interleaver { .. } | Self::CrcGenerator { .. } => BlockCategory::Coding,
            Self::GrayMapper { .. } | Self::ConstellationMapper { .. } | Self::DifferentialEncoder => BlockCategory::Mapping,
            Self::PskModulator { .. } | Self::QamModulator { .. } | Self::FskModulator { .. } |
            Self::OfdmModulator { .. } | Self::DsssSpread { .. } | Self::FhssHop { .. } | Self::CssModulator { .. } => BlockCategory::Modulation,
            Self::FirFilter { .. } | Self::IirFilter { .. } | Self::PulseShaper { .. } | Self::MatchedFilter => BlockCategory::Filtering,
            Self::Upsampler { .. } | Self::Downsampler { .. } | Self::RationalResampler { .. } | Self::PolyphaseResampler { .. } => BlockCategory::RateConversion,
            Self::PreambleInsert { .. } | Self::SyncWordInsert { .. } | Self::FrameBuilder { .. } | Self::TdmaFramer { .. } => BlockCategory::Synchronization,
            Self::AwgnChannel { .. } | Self::FadingChannel { .. } | Self::FrequencyOffset { .. } | Self::IqImbalance { .. } => BlockCategory::Impairments,
            Self::PskDemodulator { .. } | Self::QamDemodulator { .. } | Self::FskDemodulator { .. } |
            Self::Agc { .. } | Self::TimingRecovery { .. } | Self::CarrierRecovery { .. } | Self::Equalizer { .. } => BlockCategory::Recovery,
            Self::IqOutput | Self::BitOutput | Self::FileOutput { .. } | Self::Split { .. } | Self::Merge { .. } | Self::IqSplit | Self::IqMerge => BlockCategory::Output,
        }
    }

    pub fn num_inputs(&self) -> u32 {
        match self {
            Self::BitSource { .. } | Self::SymbolSource { .. } | Self::FileSource { .. } => 0,
            Self::Merge { num_inputs } => *num_inputs,
            Self::IqMerge => 2,
            _ => 1,
        }
    }

    pub fn num_outputs(&self) -> u32 {
        match self {
            Self::IqOutput | Self::BitOutput | Self::FileOutput { .. } => 0,
            Self::Split { num_outputs } => *num_outputs,
            Self::IqSplit => 2,
            _ => 1,
        }
    }
}

// Supporting enums for block parameters
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FecType {
    Convolutional,
    Turbo,
    Ldpc,
    ReedSolomon,
    Hamming,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CrcType {
    Crc8,
    Crc16Ccitt,
    Crc16Ibm,
    Crc32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConstellationType {
    Bpsk,
    Qpsk,
    Psk8,
    Qam16,
    Qam64,
    Qam256,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FilterType {
    Lowpass,
    Highpass,
    Bandpass,
    Bandstop,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum IirDesign {
    Butterworth,
    Chebyshev1,
    Chebyshev2,
    Bessel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PulseShape {
    RootRaisedCosine,
    RaisedCosine,
    Gaussian,
    Rectangular,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FadingModel {
    Rayleigh,
    Rician,
    Nakagami,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AgcMode {
    Fast,
    Slow,
    Adaptive,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TimingAlgo {
    EarlyLate,
    Gardner,
    MuellerMuller,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CarrierAlgo {
    CostasLoop,
    DecisionDirected,
    PilotAided,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EqualizerType {
    Lms,
    Rls,
    Cma,
    Dfe,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutputFormat {
    ComplexFloat32,
    ComplexFloat64,
    ComplexInt16,
    ComplexInt8,
}

/// A block in the pipeline
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PipelineBlock {
    pub id: BlockId,
    pub block_type: BlockType,
    #[serde(skip)]
    pub position: Pos2,
    pub name: String,
    pub enabled: bool,
}

impl PipelineBlock {
    pub fn new(id: BlockId, block_type: BlockType, position: Pos2) -> Self {
        let name = block_type.name().to_string();
        Self {
            id,
            block_type,
            position,
            name,
            enabled: true,
        }
    }
}

/// Connection between two blocks
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Connection {
    pub from_block: BlockId,
    pub from_port: u32,
    pub to_block: BlockId,
    pub to_port: u32,
}

/// The complete pipeline
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Pipeline {
    pub name: String,
    pub description: String,
    pub blocks: HashMap<BlockId, PipelineBlock>,
    pub connections: Vec<Connection>,
    #[serde(default)]
    pub layout_mode: LayoutMode,
    #[serde(skip)]
    next_id: BlockId,
}

/// What parts of a unified spec to load
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadMode {
    TxOnly,
    RxOnly,
    ChannelOnly,
    Loopback,  // TX → Channel → RX
    Legacy,    // Old format with single 'pipeline' section
}

/// Connection line style for visual routing
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ConnectionStyle {
    /// Smooth cubic bezier curves (default)
    #[default]
    Bezier,
    /// Direct straight line
    Straight,
    /// Right-angle (Manhattan) routing
    Orthogonal,
    /// 45-degree angled routing
    Angled,
}

impl ConnectionStyle {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Bezier => "Curved",
            Self::Straight => "Straight",
            Self::Orthogonal => "Right-angle",
            Self::Angled => "Angled",
        }
    }

    pub fn all() -> &'static [ConnectionStyle] {
        &[Self::Bezier, Self::Straight, Self::Orthogonal, Self::Angled]
    }
}

/// Selection mode for multi-select on canvas
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SelectionMode {
    /// No selection in progress
    #[default]
    None,
    /// Rectangular marquee selection (click and drag)
    Rectangle,
    /// Freeform lasso selection (click and draw path)
    Lasso,
}

/// Layout mode for auto-arranging blocks
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum LayoutMode {
    /// Topological flow layout (follows signal flow)
    Flow,
    /// Fit all blocks to visible canvas
    FitToView,
    /// Compact layout minimizing whitespace
    Compact,
    /// Simple grid arrangement (default)
    #[default]
    Grid,
}

impl LayoutMode {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Flow => "Flow (Topological)",
            Self::FitToView => "Fit to View",
            Self::Compact => "Compact",
            Self::Grid => "Grid",
        }
    }

    pub fn description(&self) -> &'static str {
        match self {
            Self::Flow => "Arrange blocks following signal flow",
            Self::FitToView => "Scale and center to fit canvas",
            Self::Compact => "Minimize spacing between blocks",
            Self::Grid => "Simple rows and columns",
        }
    }

    pub fn all() -> &'static [LayoutMode] {
        &[Self::Flow, Self::FitToView, Self::Compact, Self::Grid]
    }
}

impl Pipeline {
    pub fn new() -> Self {
        Self {
            name: "New Pipeline".to_string(),
            description: String::new(),
            blocks: HashMap::new(),
            connections: Vec::new(),
            layout_mode: LayoutMode::default(),
            next_id: 1,
        }
    }

    /// Load pipeline from YAML string
    /// Supports both direct Pipeline format and unified waveform spec format
    pub fn from_yaml(yaml: &str) -> Result<Self, String> {
        // First try parsing as unified waveform spec format
        if let Ok(value) = serde_yaml::from_str::<serde_yaml::Value>(yaml) {
            // Check if this is a unified spec (has 'waveform' and 'pipeline' sections)
            if value.get("waveform").is_some() && value.get("pipeline").is_some() {
                return Self::from_unified_spec(&value);
            }
        }

        // Fall back to direct Pipeline format
        let mut pipeline: Pipeline = serde_yaml::from_str(yaml)
            .map_err(|e| format!("Failed to parse YAML: {}", e))?;
        pipeline.fix_after_load();
        Ok(pipeline)
    }

    /// Load pipeline from YAML with specific mode
    pub fn from_yaml_with_mode(yaml: &str, mode: LoadMode) -> Result<Self, String> {
        if let Ok(value) = serde_yaml::from_str::<serde_yaml::Value>(yaml) {
            // Check if this is a unified spec with tx/rx sections
            if value.get("tx").is_some() || value.get("rx").is_some() {
                return Self::from_unified_spec_v2(&value, mode);
            }
            // Check for legacy unified spec with 'pipeline' section
            if value.get("waveform").is_some() && value.get("pipeline").is_some() {
                return Self::from_unified_spec(&value);
            }
        }

        // Fall back to direct Pipeline format
        let mut pipeline: Pipeline = serde_yaml::from_str(yaml)
            .map_err(|e| format!("Failed to parse YAML: {}", e))?;
        pipeline.fix_after_load();
        Ok(pipeline)
    }

    /// Detect what sections are available in a spec
    pub fn detect_spec_sections(yaml: &str) -> (bool, bool, bool, bool) {
        // Returns (has_tx, has_rx, has_channel, has_legacy_pipeline)
        if let Ok(value) = serde_yaml::from_str::<serde_yaml::Value>(yaml) {
            let has_tx = value.get("tx").is_some();
            let has_rx = value.get("rx").is_some();
            let has_channel = value.get("channel").is_some();
            let has_legacy = value.get("pipeline").is_some();
            return (has_tx, has_rx, has_channel, has_legacy);
        }
        (false, false, false, false)
    }

    /// Parse unified waveform spec v2 format (with tx/rx/channel sections)
    fn from_unified_spec_v2(value: &serde_yaml::Value, mode: LoadMode) -> Result<Self, String> {
        let mut pipeline = Pipeline::new();

        // Extract name and description from waveform section
        if let Some(waveform) = value.get("waveform") {
            if let Some(name) = waveform.get("name").and_then(|v| v.as_str()) {
                let suffix = match mode {
                    LoadMode::TxOnly => " TX",
                    LoadMode::RxOnly => " RX",
                    LoadMode::ChannelOnly => " Channel",
                    LoadMode::Loopback => " Loopback",
                    LoadMode::Legacy => "",
                };
                pipeline.name = format!("{}{}", name, suffix);
            }
            if let Some(desc) = waveform.get("description").and_then(|v| v.as_str()) {
                pipeline.description = desc.to_string();
            }
        }

        // Load sections based on mode
        match mode {
            LoadMode::TxOnly => {
                if let Some(tx) = value.get("tx") {
                    Self::parse_pipeline_section(&mut pipeline, tx)?;
                } else {
                    return Err("No 'tx' section found in spec".to_string());
                }
            }
            LoadMode::RxOnly => {
                if let Some(rx) = value.get("rx") {
                    Self::parse_pipeline_section(&mut pipeline, rx)?;
                } else {
                    return Err("No 'rx' section found in spec".to_string());
                }
            }
            LoadMode::ChannelOnly => {
                if let Some(channel) = value.get("channel") {
                    Self::parse_pipeline_section(&mut pipeline, channel)?;
                } else {
                    return Err("No 'channel' section found in spec".to_string());
                }
            }
            LoadMode::Loopback => {
                // Load TX
                if let Some(tx) = value.get("tx") {
                    Self::parse_pipeline_section(&mut pipeline, tx)?;
                }
                // Load Channel
                if let Some(channel) = value.get("channel") {
                    Self::parse_pipeline_section(&mut pipeline, channel)?;
                }
                // Load RX
                if let Some(rx) = value.get("rx") {
                    Self::parse_pipeline_section(&mut pipeline, rx)?;
                }
                // Connect TX → Channel → RX
                // Find last TX block and first Channel block
                let tx_last = pipeline.blocks.iter()
                    .filter(|(id, _)| **id < 100)
                    .max_by_key(|(id, _)| *id)
                    .map(|(id, _)| *id);
                let channel_first = pipeline.blocks.iter()
                    .filter(|(id, _)| **id >= 200 && **id < 300)
                    .min_by_key(|(id, _)| *id)
                    .map(|(id, _)| *id);
                let channel_last = pipeline.blocks.iter()
                    .filter(|(id, _)| **id >= 200 && **id < 300)
                    .max_by_key(|(id, _)| *id)
                    .map(|(id, _)| *id);
                let rx_first = pipeline.blocks.iter()
                    .filter(|(id, _)| **id >= 100 && **id < 200)
                    .min_by_key(|(id, _)| *id)
                    .map(|(id, _)| *id);

                // Connect TX → Channel
                if let (Some(tx_id), Some(ch_id)) = (tx_last, channel_first) {
                    pipeline.connections.push(Connection {
                        from_block: tx_id, from_port: 0,
                        to_block: ch_id, to_port: 0,
                    });
                }
                // Connect Channel → RX
                if let (Some(ch_id), Some(rx_id)) = (channel_last, rx_first) {
                    pipeline.connections.push(Connection {
                        from_block: ch_id, from_port: 0,
                        to_block: rx_id, to_port: 0,
                    });
                }
                // If no channel, connect TX → RX directly
                if channel_first.is_none() {
                    if let (Some(tx_id), Some(rx_id)) = (tx_last, rx_first) {
                        pipeline.connections.push(Connection {
                            from_block: tx_id, from_port: 0,
                            to_block: rx_id, to_port: 0,
                        });
                    }
                }
            }
            LoadMode::Legacy => {
                // Handled by from_unified_spec
                return Err("Use from_unified_spec for legacy format".to_string());
            }
        }

        pipeline.fix_after_load();
        Ok(pipeline)
    }

    /// Parse a pipeline section (tx, rx, or channel)
    fn parse_pipeline_section(pipeline: &mut Pipeline, section: &serde_yaml::Value) -> Result<(), String> {
        // Parse blocks
        if let Some(blocks) = section.get("blocks").and_then(|v| v.as_sequence()) {
            for block_val in blocks {
                if let (Some(id), Some(block_type_name)) = (
                    block_val.get("id").and_then(|v| v.as_u64()),
                    block_val.get("type").and_then(|v| v.as_str()),
                ) {
                    let name = block_val.get("name")
                        .and_then(|v| v.as_str())
                        .unwrap_or(block_type_name)
                        .to_string();
                    let enabled = block_val.get("enabled")
                        .and_then(|v| v.as_bool())
                        .unwrap_or(true);

                    if let Some(block_type) = Self::parse_block_type(block_type_name, block_val) {
                        let mut block = PipelineBlock::new(id as u32, block_type, Pos2::ZERO);
                        block.name = name;
                        block.enabled = enabled;
                        pipeline.blocks.insert(id as u32, block);
                    }
                }
            }
        }

        // Parse connections
        if let Some(connections) = section.get("connections").and_then(|v| v.as_sequence()) {
            for conn_val in connections {
                if let (Some(from), Some(to)) = (
                    conn_val.get("from").and_then(|v| v.as_sequence()),
                    conn_val.get("to").and_then(|v| v.as_sequence()),
                ) {
                    if from.len() >= 2 && to.len() >= 2 {
                        if let (Some(fb), Some(fp), Some(tb), Some(tp)) = (
                            from[0].as_u64(),
                            from[1].as_u64(),
                            to[0].as_u64(),
                            to[1].as_u64(),
                        ) {
                            pipeline.connections.push(Connection {
                                from_block: fb as u32,
                                from_port: fp as u32,
                                to_block: tb as u32,
                                to_port: tp as u32,
                            });
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Parse unified waveform spec format (legacy - single pipeline section)
    fn from_unified_spec(value: &serde_yaml::Value) -> Result<Self, String> {
        let mut pipeline = Pipeline::new();

        // Extract name and description from waveform section
        if let Some(waveform) = value.get("waveform") {
            if let Some(name) = waveform.get("name").and_then(|v| v.as_str()) {
                pipeline.name = name.to_string();
            }
            if let Some(desc) = waveform.get("description").and_then(|v| v.as_str()) {
                pipeline.description = desc.to_string();
            }
        }

        // Extract blocks and connections from pipeline section
        if let Some(pipeline_section) = value.get("pipeline") {
            // Parse blocks
            if let Some(blocks) = pipeline_section.get("blocks").and_then(|v| v.as_sequence()) {
                for block_val in blocks {
                    if let (Some(id), Some(block_type_name)) = (
                        block_val.get("id").and_then(|v| v.as_u64()),
                        block_val.get("type").and_then(|v| v.as_str()),
                    ) {
                        let name = block_val.get("name")
                            .and_then(|v| v.as_str())
                            .unwrap_or(block_type_name)
                            .to_string();
                        let enabled = block_val.get("enabled")
                            .and_then(|v| v.as_bool())
                            .unwrap_or(true);

                        // Create block type from name and parameters
                        if let Some(block_type) = Self::parse_block_type(block_type_name, block_val) {
                            let mut block = PipelineBlock::new(id as u32, block_type, Pos2::ZERO);
                            block.name = name;
                            block.enabled = enabled;
                            pipeline.blocks.insert(id as u32, block);
                        }
                    }
                }
            }

            // Parse connections
            if let Some(connections) = pipeline_section.get("connections").and_then(|v| v.as_sequence()) {
                for conn_val in connections {
                    if let (Some(from), Some(to)) = (
                        conn_val.get("from").and_then(|v| v.as_sequence()),
                        conn_val.get("to").and_then(|v| v.as_sequence()),
                    ) {
                        if from.len() >= 2 && to.len() >= 2 {
                            if let (Some(fb), Some(fp), Some(tb), Some(tp)) = (
                                from[0].as_u64(),
                                from[1].as_u64(),
                                to[0].as_u64(),
                                to[1].as_u64(),
                            ) {
                                pipeline.connections.push(Connection {
                                    from_block: fb as u32,
                                    from_port: fp as u32,
                                    to_block: tb as u32,
                                    to_port: tp as u32,
                                });
                            }
                        }
                    }
                }
            }
        }

        pipeline.fix_after_load();
        Ok(pipeline)
    }

    /// Parse a block type from YAML
    fn parse_block_type(type_name: &str, block_val: &serde_yaml::Value) -> Option<BlockType> {
        match type_name {
            "Bit Source" => Some(BlockType::BitSource {
                pattern: block_val.get("pattern")
                    .and_then(|v| v.as_str())
                    .unwrap_or("random")
                    .to_string(),
            }),
            "Symbol Source" => Some(BlockType::SymbolSource {
                alphabet_size: block_val.get("alphabet_size")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(4) as u32,
            }),
            "File Source" => Some(BlockType::FileSource {
                path: block_val.get("path")
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_string(),
            }),
            "Scrambler" => Some(BlockType::Scrambler {
                polynomial: block_val.get("polynomial")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(0x1D) as u32,
                seed: block_val.get("seed")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(0xFF) as u32,
            }),
            "FEC Encoder" => Some(BlockType::FecEncoder {
                code_type: FecType::Hamming,
                rate: block_val.get("rate")
                    .and_then(|v| v.as_str())
                    .unwrap_or("1/2")
                    .to_string(),
            }),
            "Interleaver" => Some(BlockType::Interleaver {
                rows: block_val.get("rows")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(8) as u32,
                cols: block_val.get("cols")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(8) as u32,
            }),
            "Gray Mapper" => Some(BlockType::GrayMapper {
                bits_per_symbol: block_val.get("bits_per_symbol")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(2) as u32,
            }),
            "PSK Modulator" => Some(BlockType::PskModulator {
                order: block_val.get("order")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(2) as u32,
            }),
            "QAM Modulator" => Some(BlockType::QamModulator {
                order: block_val.get("order")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(16) as u32,
            }),
            "FSK Modulator" => Some(BlockType::FskModulator {
                deviation_hz: block_val.get("deviation_hz")
                    .and_then(|v| v.as_f64())
                    .unwrap_or(2500.0) as f32,
                order: block_val.get("order")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(2) as u32,
            }),
            "CSS Modulator" | "LoRa Modulator" => Some(BlockType::CssModulator {
                sf: block_val.get("spreading_factor")
                    .or_else(|| block_val.get("sf"))
                    .and_then(|v| v.as_u64())
                    .unwrap_or(7) as u8,
                bw_hz: block_val.get("bandwidth_hz")
                    .or_else(|| block_val.get("bw_hz"))
                    .and_then(|v| v.as_u64())
                    .unwrap_or(125000) as u32,
            }),
            "RRC Filter" | "Pulse Shaper" => Some(BlockType::PulseShaper {
                shape: PulseShape::RootRaisedCosine,
                rolloff: block_val.get("rolloff")
                    .and_then(|v| v.as_f64())
                    .unwrap_or(0.35) as f32,
                span: block_val.get("span_symbols")
                    .or_else(|| block_val.get("span"))
                    .and_then(|v| v.as_u64())
                    .unwrap_or(8) as u32,
            }),
            "Gaussian Filter" => Some(BlockType::PulseShaper {
                shape: PulseShape::Gaussian,
                rolloff: block_val.get("bt_product")
                    .or_else(|| block_val.get("rolloff"))
                    .and_then(|v| v.as_f64())
                    .unwrap_or(0.5) as f32,
                span: block_val.get("span_symbols")
                    .or_else(|| block_val.get("span"))
                    .and_then(|v| v.as_u64())
                    .unwrap_or(4) as u32,
            }),
            "Tone Generator" | "Carrier Generator" => {
                // Use a bit source with fixed pattern as placeholder
                // (no dedicated tone generator block)
                log::info!("Tone Generator mapped to Bit Source placeholder");
                Some(BlockType::BitSource { pattern: "tone".to_string() })
            },
            "AWGN Channel" => Some(BlockType::AwgnChannel {
                snr_db: block_val.get("snr_db")
                    .and_then(|v| v.as_f64())
                    .unwrap_or(20.0) as f32,
            }),
            "Frequency Offset" => Some(BlockType::FrequencyOffset {
                offset_hz: block_val.get("offset_hz")
                    .and_then(|v| v.as_f64())
                    .unwrap_or(0.0) as f32,
            }),
            "File Sink" | "File Output" => Some(BlockType::FileOutput {
                path: block_val.get("path")
                    .and_then(|v| v.as_str())
                    .unwrap_or("output.iq")
                    .to_string(),
                format: OutputFormat::ComplexFloat32,
            }),
            "AGC" => {
                let mode_str = block_val.get("mode")
                    .and_then(|v| v.as_str())
                    .unwrap_or("Adaptive");
                let mode = match mode_str {
                    "Fast" => AgcMode::Fast,
                    "Slow" => AgcMode::Slow,
                    _ => AgcMode::Adaptive,
                };
                Some(BlockType::Agc {
                    mode,
                    target_db: block_val.get("target_db")
                        .and_then(|v| v.as_f64())
                        .unwrap_or(0.0) as f32,
                })
            }
            "Timing Recovery" => {
                let algo_str = block_val.get("algorithm")
                    .and_then(|v| v.as_str())
                    .unwrap_or("Gardner");
                let algorithm = match algo_str {
                    "EarlyLate" => TimingAlgo::EarlyLate,
                    "MuellerMuller" => TimingAlgo::MuellerMuller,
                    _ => TimingAlgo::Gardner,
                };
                Some(BlockType::TimingRecovery {
                    algorithm,
                    loop_bw: block_val.get("loop_bw")
                        .and_then(|v| v.as_f64())
                        .unwrap_or(0.01) as f32,
                })
            }
            "Carrier Recovery" => {
                let algo_str = block_val.get("algorithm")
                    .and_then(|v| v.as_str())
                    .unwrap_or("CostasLoop");
                let algorithm = match algo_str {
                    "DecisionDirected" => CarrierAlgo::DecisionDirected,
                    "PilotAided" => CarrierAlgo::PilotAided,
                    _ => CarrierAlgo::CostasLoop,
                };
                Some(BlockType::CarrierRecovery {
                    algorithm,
                    loop_bw: block_val.get("loop_bw")
                        .and_then(|v| v.as_f64())
                        .unwrap_or(0.01) as f32,
                })
            }
            "PSK Demodulator" => Some(BlockType::PskDemodulator {
                order: block_val.get("order")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(2) as u32,
            }),
            "QAM Demodulator" => Some(BlockType::QamDemodulator {
                order: block_val.get("order")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(16) as u32,
            }),
            "FSK Demodulator" => Some(BlockType::FskDemodulator {
                order: block_val.get("order")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(2) as u32,
            }),
            "Bit Output" => Some(BlockType::BitOutput),
            "IQ Output" => Some(BlockType::IqOutput),
            _ => {
                log::warn!("Unknown block type: {}", type_name);
                None
            }
        }
    }

    /// Save pipeline to YAML string (structured format for re-loading)
    pub fn to_yaml_structured(&self) -> String {
        serde_yaml::to_string(self).unwrap_or_else(|e| format!("# Error: {}", e))
    }

    /// Fix up the pipeline after deserialization
    fn fix_after_load(&mut self) {
        // Compute next_id from existing blocks
        self.next_id = self.blocks.keys().max().map(|m| m + 1).unwrap_or(1);

        // Auto-layout blocks that have default position
        self.auto_layout();
    }

    pub fn add_block(&mut self, block_type: BlockType, position: Pos2) -> BlockId {
        let id = self.next_id;
        self.next_id += 1;
        let block = PipelineBlock::new(id, block_type, position);
        self.blocks.insert(id, block);
        id
    }

    /// Find an empty position for a new block, avoiding overlap with existing blocks
    pub fn find_empty_position(&self, vertical: bool) -> Pos2 {
        const BLOCK_WIDTH: f32 = 140.0;
        const BLOCK_HEIGHT: f32 = 80.0;
        const SPACING: f32 = 20.0;
        const START_X: f32 = 50.0;
        const START_Y: f32 = 50.0;

        if self.blocks.is_empty() {
            return Pos2::new(START_X, START_Y);
        }

        if vertical {
            // Find the lowest Y position among existing blocks
            let max_y = self.blocks.values()
                .map(|b| b.position.y)
                .fold(f32::NEG_INFINITY, f32::max);
            // Place new block below the lowest existing block
            Pos2::new(START_X, max_y + BLOCK_HEIGHT + SPACING)
        } else {
            // Find the rightmost X position among existing blocks
            let max_x = self.blocks.values()
                .map(|b| b.position.x)
                .fold(f32::NEG_INFINITY, f32::max);
            // Place new block to the right of the rightmost existing block
            Pos2::new(max_x + BLOCK_WIDTH + SPACING, START_Y)
        }
    }

    /// Find the best block to auto-connect a new block to
    /// Returns the block that has no outgoing connections and is furthest along the layout direction
    pub fn find_auto_connect_source(&self, vertical: bool) -> Option<BlockId> {
        // Find blocks that have outputs but no outgoing connections
        let blocks_with_free_outputs: Vec<_> = self.blocks.iter()
            .filter(|(id, block)| {
                // Block must have at least one output
                if block.block_type.num_outputs() == 0 {
                    return false;
                }
                // Check if any output port is unconnected
                let num_outputs = block.block_type.num_outputs();
                for port in 0..num_outputs {
                    let has_connection = self.connections.iter()
                        .any(|c| c.from_block == **id && c.from_port == port);
                    if !has_connection {
                        return true;
                    }
                }
                false
            })
            .collect();

        if blocks_with_free_outputs.is_empty() {
            return None;
        }

        // Find the block furthest along the layout direction
        let best = if vertical {
            // For vertical layout, find the block with highest Y (lowest on screen)
            blocks_with_free_outputs.iter()
                .max_by(|a, b| a.1.position.y.partial_cmp(&b.1.position.y).unwrap())
        } else {
            // For horizontal layout, find the block with highest X (rightmost)
            blocks_with_free_outputs.iter()
                .max_by(|a, b| a.1.position.x.partial_cmp(&b.1.position.x).unwrap())
        };

        best.map(|(id, _)| **id)
    }

    /// Find the first free output port on a block
    pub fn find_free_output_port(&self, block_id: BlockId) -> Option<u32> {
        let block = self.blocks.get(&block_id)?;
        let num_outputs = block.block_type.num_outputs();

        for port in 0..num_outputs {
            let has_connection = self.connections.iter()
                .any(|c| c.from_block == block_id && c.from_port == port);
            if !has_connection {
                return Some(port);
            }
        }
        None
    }

    pub fn remove_block(&mut self, id: BlockId) {
        self.blocks.remove(&id);
        self.connections.retain(|c| c.from_block != id && c.to_block != id);
    }

    pub fn connect(&mut self, from_block: BlockId, from_port: u32, to_block: BlockId, to_port: u32) {
        // Remove existing connection to this input port
        self.connections.retain(|c| !(c.to_block == to_block && c.to_port == to_port));

        self.connections.push(Connection {
            from_block,
            from_port,
            to_block,
            to_port,
        });
    }

    pub fn disconnect(&mut self, from_block: BlockId, from_port: u32, to_block: BlockId, to_port: u32) {
        self.connections.retain(|c| {
            !(c.from_block == from_block && c.from_port == from_port &&
              c.to_block == to_block && c.to_port == to_port)
        });
    }

    /// Generate YAML specification
    pub fn to_yaml(&self) -> String {
        let mut yaml = format!(
            r#"# Pipeline Waveform Specification
# Generated by R4W Pipeline Builder

name: "{}"
description: "{}"

pipeline:
  blocks:
"#,
            self.name, self.description
        );

        // Sort blocks by ID for consistent output
        let mut block_ids: Vec<_> = self.blocks.keys().collect();
        block_ids.sort();

        for id in block_ids {
            let block = &self.blocks[id];
            yaml.push_str(&format!("    - id: {}\n", block.id));
            yaml.push_str(&format!("      name: \"{}\"\n", block.name));
            yaml.push_str(&format!("      type: \"{}\"\n", block.block_type.name()));
            yaml.push_str(&format!("      enabled: {}\n", block.enabled));
            yaml.push_str(&self.block_params_to_yaml(&block.block_type));
            yaml.push('\n');
        }

        yaml.push_str("  connections:\n");
        for conn in &self.connections {
            yaml.push_str(&format!(
                "    - from: [{}, {}]\n      to: [{}, {}]\n",
                conn.from_block, conn.from_port, conn.to_block, conn.to_port
            ));
        }

        yaml
    }

    fn block_params_to_yaml(&self, block_type: &BlockType) -> String {
        match block_type {
            BlockType::BitSource { pattern } => format!("      pattern: \"{}\"\n", pattern),
            BlockType::SymbolSource { alphabet_size } => format!("      alphabet_size: {}\n", alphabet_size),
            BlockType::FileSource { path } => format!("      path: \"{}\"\n", path),
            BlockType::Scrambler { polynomial, seed } => {
                format!("      polynomial: 0x{:X}\n      seed: 0x{:X}\n", polynomial, seed)
            }
            BlockType::FecEncoder { code_type, rate } => {
                format!("      code_type: {:?}\n      rate: \"{}\"\n", code_type, rate)
            }
            BlockType::Interleaver { rows, cols } => {
                format!("      rows: {}\n      cols: {}\n", rows, cols)
            }
            BlockType::CrcGenerator { crc_type } => format!("      crc_type: {:?}\n", crc_type),
            BlockType::GrayMapper { bits_per_symbol } => format!("      bits_per_symbol: {}\n", bits_per_symbol),
            BlockType::ConstellationMapper { constellation } => format!("      constellation: {:?}\n", constellation),
            BlockType::PskModulator { order } => format!("      order: {}\n", order),
            BlockType::QamModulator { order } => format!("      order: {}\n", order),
            BlockType::FskModulator { deviation_hz, order } => {
                format!("      deviation_hz: {}\n      order: {}\n", deviation_hz, order)
            }
            BlockType::OfdmModulator { fft_size, cp_len, data_carriers } => {
                format!("      fft_size: {}\n      cp_len: {}\n      data_carriers: {}\n", fft_size, cp_len, data_carriers)
            }
            BlockType::DsssSpread { chips_per_symbol, code_type } => {
                format!("      chips_per_symbol: {}\n      code_type: \"{}\"\n", chips_per_symbol, code_type)
            }
            BlockType::FhssHop { num_channels, hop_rate } => {
                format!("      num_channels: {}\n      hop_rate: {}\n", num_channels, hop_rate)
            }
            BlockType::CssModulator { sf, bw_hz } => {
                format!("      spreading_factor: {}\n      bandwidth_hz: {}\n", sf, bw_hz)
            }
            BlockType::FirFilter { filter_type, cutoff_hz, num_taps } => {
                format!("      filter_type: {:?}\n      cutoff_hz: {}\n      num_taps: {}\n",
                    filter_type, cutoff_hz, num_taps)
            }
            BlockType::IirFilter { design, cutoff_hz, order } => {
                format!("      design: {:?}\n      cutoff_hz: {}\n      order: {}\n", design, cutoff_hz, order)
            }
            BlockType::PulseShaper { shape, rolloff, span } => {
                format!("      shape: {:?}\n      rolloff: {}\n      span: {}\n", shape, rolloff, span)
            }
            BlockType::Upsampler { factor } => format!("      factor: {}\n", factor),
            BlockType::Downsampler { factor } => format!("      factor: {}\n", factor),
            BlockType::RationalResampler { up, down } => {
                format!("      up: {}\n      down: {}\n", up, down)
            }
            BlockType::PolyphaseResampler { up, down, taps_per_phase } => {
                format!("      up: {}\n      down: {}\n      taps_per_phase: {}\n", up, down, taps_per_phase)
            }
            BlockType::PreambleInsert { pattern, length } => {
                format!("      pattern: \"{}\"\n      length: {}\n", pattern, length)
            }
            BlockType::SyncWordInsert { word } => format!("      word: \"{}\"\n", word),
            BlockType::FrameBuilder { header_bits, payload_bits } => {
                format!("      header_bits: {}\n      payload_bits: {}\n", header_bits, payload_bits)
            }
            BlockType::TdmaFramer { slots, slot_ms } => {
                format!("      slots: {}\n      slot_ms: {}\n", slots, slot_ms)
            }
            BlockType::AwgnChannel { snr_db } => format!("      snr_db: {}\n", snr_db),
            BlockType::FadingChannel { model, doppler_hz } => {
                format!("      model: {:?}\n      doppler_hz: {}\n", model, doppler_hz)
            }
            BlockType::FrequencyOffset { offset_hz } => format!("      offset_hz: {}\n", offset_hz),
            BlockType::IqImbalance { gain_db, phase_deg } => {
                format!("      gain_db: {}\n      phase_deg: {}\n", gain_db, phase_deg)
            }
            BlockType::Agc { mode, target_db } => {
                format!("      mode: {:?}\n      target_db: {}\n", mode, target_db)
            }
            BlockType::TimingRecovery { algorithm, loop_bw } => {
                format!("      algorithm: {:?}\n      loop_bw: {}\n", algorithm, loop_bw)
            }
            BlockType::CarrierRecovery { algorithm, loop_bw } => {
                format!("      algorithm: {:?}\n      loop_bw: {}\n", algorithm, loop_bw)
            }
            BlockType::Equalizer { eq_type, taps, mu } => {
                format!("      type: {:?}\n      taps: {}\n      mu: {}\n", eq_type, taps, mu)
            }
            BlockType::FileOutput { path, format } => {
                format!("      path: \"{}\"\n      format: {:?}\n", path, format)
            }
            BlockType::Split { num_outputs } => format!("      num_outputs: {}\n", num_outputs),
            BlockType::Merge { num_inputs } => format!("      num_inputs: {}\n", num_inputs),
            BlockType::PskDemodulator { order } => format!("      order: {}\n", order),
            BlockType::QamDemodulator { order } => format!("      order: {}\n", order),
            BlockType::FskDemodulator { order } => format!("      order: {}\n", order),
            _ => String::new(),
        }
    }

    /// Create pipeline from preset template
    pub fn from_preset(preset: PipelinePreset) -> Self {
        let mut pipeline = Pipeline::new();
        pipeline.name = preset.name().to_string();
        pipeline.description = preset.description().to_string();

        match preset {
            PipelinePreset::BpskTx => {
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 150.0));
                let mod_ = pipeline.add_block(BlockType::PskModulator { order: 2 }, Pos2::new(200.0, 150.0));
                let up = pipeline.add_block(BlockType::Upsampler { factor: 4 }, Pos2::new(350.0, 150.0));
                let rrc = pipeline.add_block(BlockType::PulseShaper { shape: PulseShape::RootRaisedCosine, rolloff: 0.35, span: 8 }, Pos2::new(500.0, 150.0));
                let out = pipeline.add_block(BlockType::IqOutput, Pos2::new(650.0, 150.0));

                pipeline.connect(src, 0, mod_, 0);
                pipeline.connect(mod_, 0, up, 0);
                pipeline.connect(up, 0, rrc, 0);
                pipeline.connect(rrc, 0, out, 0);
            }
            PipelinePreset::QpskTx => {
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 150.0));
                let gray = pipeline.add_block(BlockType::GrayMapper { bits_per_symbol: 2 }, Pos2::new(200.0, 150.0));
                let mod_ = pipeline.add_block(BlockType::PskModulator { order: 4 }, Pos2::new(350.0, 150.0));
                let up = pipeline.add_block(BlockType::Upsampler { factor: 4 }, Pos2::new(500.0, 150.0));
                let rrc = pipeline.add_block(BlockType::PulseShaper { shape: PulseShape::RootRaisedCosine, rolloff: 0.35, span: 8 }, Pos2::new(650.0, 150.0));
                let out = pipeline.add_block(BlockType::IqOutput, Pos2::new(800.0, 150.0));

                pipeline.connect(src, 0, gray, 0);
                pipeline.connect(gray, 0, mod_, 0);
                pipeline.connect(mod_, 0, up, 0);
                pipeline.connect(up, 0, rrc, 0);
                pipeline.connect(rrc, 0, out, 0);
            }
            PipelinePreset::Qam16Tx => {
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 150.0));
                let scram = pipeline.add_block(BlockType::Scrambler { polynomial: 0x48, seed: 0xFF }, Pos2::new(200.0, 150.0));
                let fec = pipeline.add_block(BlockType::FecEncoder { code_type: FecType::Convolutional, rate: "1/2".into() }, Pos2::new(350.0, 150.0));
                let gray = pipeline.add_block(BlockType::GrayMapper { bits_per_symbol: 4 }, Pos2::new(500.0, 150.0));
                let mod_ = pipeline.add_block(BlockType::QamModulator { order: 16 }, Pos2::new(650.0, 150.0));
                let up = pipeline.add_block(BlockType::Upsampler { factor: 4 }, Pos2::new(800.0, 150.0));
                let rrc = pipeline.add_block(BlockType::PulseShaper { shape: PulseShape::RootRaisedCosine, rolloff: 0.25, span: 8 }, Pos2::new(950.0, 150.0));
                let out = pipeline.add_block(BlockType::IqOutput, Pos2::new(1100.0, 150.0));

                pipeline.connect(src, 0, scram, 0);
                pipeline.connect(scram, 0, fec, 0);
                pipeline.connect(fec, 0, gray, 0);
                pipeline.connect(gray, 0, mod_, 0);
                pipeline.connect(mod_, 0, up, 0);
                pipeline.connect(up, 0, rrc, 0);
                pipeline.connect(rrc, 0, out, 0);
            }
            PipelinePreset::LoRaTx => {
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 150.0));
                let scram = pipeline.add_block(BlockType::Scrambler { polynomial: 0x12, seed: 0xFF }, Pos2::new(200.0, 150.0));
                let inter = pipeline.add_block(BlockType::Interleaver { rows: 4, cols: 8 }, Pos2::new(350.0, 150.0));
                let gray = pipeline.add_block(BlockType::GrayMapper { bits_per_symbol: 7 }, Pos2::new(500.0, 150.0));
                let css = pipeline.add_block(BlockType::CssModulator { sf: 7, bw_hz: 125000 }, Pos2::new(650.0, 150.0));
                let out = pipeline.add_block(BlockType::IqOutput, Pos2::new(800.0, 150.0));

                pipeline.connect(src, 0, scram, 0);
                pipeline.connect(scram, 0, inter, 0);
                pipeline.connect(inter, 0, gray, 0);
                pipeline.connect(gray, 0, css, 0);
                pipeline.connect(css, 0, out, 0);
            }
            PipelinePreset::OfdmTx => {
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 150.0));
                let scram = pipeline.add_block(BlockType::Scrambler { polynomial: 0x48, seed: 0x7F }, Pos2::new(200.0, 150.0));
                let fec = pipeline.add_block(BlockType::FecEncoder { code_type: FecType::Convolutional, rate: "1/2".into() }, Pos2::new(350.0, 150.0));
                let inter = pipeline.add_block(BlockType::Interleaver { rows: 16, cols: 12 }, Pos2::new(500.0, 150.0));
                let ofdm = pipeline.add_block(BlockType::OfdmModulator { fft_size: 64, cp_len: 16, data_carriers: 48 }, Pos2::new(650.0, 150.0));
                let out = pipeline.add_block(BlockType::IqOutput, Pos2::new(800.0, 150.0));

                pipeline.connect(src, 0, scram, 0);
                pipeline.connect(scram, 0, fec, 0);
                pipeline.connect(fec, 0, inter, 0);
                pipeline.connect(inter, 0, ofdm, 0);
                pipeline.connect(ofdm, 0, out, 0);
            }
            PipelinePreset::FskTx => {
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 150.0));
                let fsk = pipeline.add_block(BlockType::FskModulator { deviation_hz: 2500.0, order: 2 }, Pos2::new(200.0, 150.0));
                let up = pipeline.add_block(BlockType::Upsampler { factor: 8 }, Pos2::new(350.0, 150.0));
                let fir = pipeline.add_block(BlockType::FirFilter { filter_type: FilterType::Lowpass, cutoff_hz: 5000.0, num_taps: 64 }, Pos2::new(500.0, 150.0));
                let out = pipeline.add_block(BlockType::IqOutput, Pos2::new(650.0, 150.0));

                pipeline.connect(src, 0, fsk, 0);
                pipeline.connect(fsk, 0, up, 0);
                pipeline.connect(up, 0, fir, 0);
                pipeline.connect(fir, 0, out, 0);
            }
            PipelinePreset::BpskTxRx => {
                // TX chain
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 100.0));
                let mod_ = pipeline.add_block(BlockType::PskModulator { order: 2 }, Pos2::new(200.0, 100.0));
                let up = pipeline.add_block(BlockType::Upsampler { factor: 4 }, Pos2::new(350.0, 100.0));
                let rrc_tx = pipeline.add_block(BlockType::PulseShaper { shape: PulseShape::RootRaisedCosine, rolloff: 0.35, span: 8 }, Pos2::new(500.0, 100.0));

                // Channel
                let awgn = pipeline.add_block(BlockType::AwgnChannel { snr_db: 10.0 }, Pos2::new(650.0, 100.0));

                // RX chain
                let rrc_rx = pipeline.add_block(BlockType::MatchedFilter, Pos2::new(50.0, 250.0));
                let agc = pipeline.add_block(BlockType::Agc { mode: AgcMode::Adaptive, target_db: -20.0 }, Pos2::new(200.0, 250.0));
                let timing = pipeline.add_block(BlockType::TimingRecovery { algorithm: TimingAlgo::Gardner, loop_bw: 0.01 }, Pos2::new(350.0, 250.0));
                let carrier = pipeline.add_block(BlockType::CarrierRecovery { algorithm: CarrierAlgo::CostasLoop, loop_bw: 0.005 }, Pos2::new(500.0, 250.0));
                let down = pipeline.add_block(BlockType::Downsampler { factor: 4 }, Pos2::new(650.0, 250.0));
                let out = pipeline.add_block(BlockType::BitOutput, Pos2::new(800.0, 250.0));

                pipeline.connect(src, 0, mod_, 0);
                pipeline.connect(mod_, 0, up, 0);
                pipeline.connect(up, 0, rrc_tx, 0);
                pipeline.connect(rrc_tx, 0, awgn, 0);
                pipeline.connect(awgn, 0, rrc_rx, 0);
                pipeline.connect(rrc_rx, 0, agc, 0);
                pipeline.connect(agc, 0, timing, 0);
                pipeline.connect(timing, 0, carrier, 0);
                pipeline.connect(carrier, 0, down, 0);
                pipeline.connect(down, 0, out, 0);
            }
            PipelinePreset::QpskTxRx => {
                // TX chain
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 100.0));
                let gray_tx = pipeline.add_block(BlockType::GrayMapper { bits_per_symbol: 2 }, Pos2::new(200.0, 100.0));
                let mod_ = pipeline.add_block(BlockType::PskModulator { order: 4 }, Pos2::new(350.0, 100.0));
                let up = pipeline.add_block(BlockType::Upsampler { factor: 4 }, Pos2::new(500.0, 100.0));
                let rrc_tx = pipeline.add_block(BlockType::PulseShaper { shape: PulseShape::RootRaisedCosine, rolloff: 0.35, span: 8 }, Pos2::new(650.0, 100.0));

                // Channel with fading
                let fading = pipeline.add_block(BlockType::FadingChannel { model: FadingModel::Rayleigh, doppler_hz: 10.0 }, Pos2::new(800.0, 100.0));
                let awgn = pipeline.add_block(BlockType::AwgnChannel { snr_db: 15.0 }, Pos2::new(950.0, 100.0));

                // RX chain
                let rrc_rx = pipeline.add_block(BlockType::MatchedFilter, Pos2::new(50.0, 250.0));
                let agc = pipeline.add_block(BlockType::Agc { mode: AgcMode::Adaptive, target_db: -20.0 }, Pos2::new(200.0, 250.0));
                let eq = pipeline.add_block(BlockType::Equalizer { eq_type: EqualizerType::Lms, taps: 11, mu: 0.01 }, Pos2::new(350.0, 250.0));
                let timing = pipeline.add_block(BlockType::TimingRecovery { algorithm: TimingAlgo::Gardner, loop_bw: 0.01 }, Pos2::new(500.0, 250.0));
                let carrier = pipeline.add_block(BlockType::CarrierRecovery { algorithm: CarrierAlgo::CostasLoop, loop_bw: 0.005 }, Pos2::new(650.0, 250.0));
                let down = pipeline.add_block(BlockType::Downsampler { factor: 4 }, Pos2::new(800.0, 250.0));
                let out = pipeline.add_block(BlockType::BitOutput, Pos2::new(950.0, 250.0));

                pipeline.connect(src, 0, gray_tx, 0);
                pipeline.connect(gray_tx, 0, mod_, 0);
                pipeline.connect(mod_, 0, up, 0);
                pipeline.connect(up, 0, rrc_tx, 0);
                pipeline.connect(rrc_tx, 0, fading, 0);
                pipeline.connect(fading, 0, awgn, 0);
                pipeline.connect(awgn, 0, rrc_rx, 0);
                pipeline.connect(rrc_rx, 0, agc, 0);
                pipeline.connect(agc, 0, eq, 0);
                pipeline.connect(eq, 0, timing, 0);
                pipeline.connect(timing, 0, carrier, 0);
                pipeline.connect(carrier, 0, down, 0);
                pipeline.connect(down, 0, out, 0);
            }
            PipelinePreset::DsssTx => {
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 150.0));
                let scram = pipeline.add_block(BlockType::Scrambler { polynomial: 0x48, seed: 0xFF }, Pos2::new(200.0, 150.0));
                let fec = pipeline.add_block(BlockType::FecEncoder { code_type: FecType::Convolutional, rate: "1/2".into() }, Pos2::new(350.0, 150.0));
                let dsss = pipeline.add_block(BlockType::DsssSpread { chips_per_symbol: 127, code_type: "Gold".into() }, Pos2::new(500.0, 150.0));
                let mod_ = pipeline.add_block(BlockType::PskModulator { order: 2 }, Pos2::new(650.0, 150.0));
                let up = pipeline.add_block(BlockType::Upsampler { factor: 4 }, Pos2::new(800.0, 150.0));
                let rrc = pipeline.add_block(BlockType::PulseShaper { shape: PulseShape::RootRaisedCosine, rolloff: 0.35, span: 8 }, Pos2::new(950.0, 150.0));
                let out = pipeline.add_block(BlockType::IqOutput, Pos2::new(1100.0, 150.0));

                pipeline.connect(src, 0, scram, 0);
                pipeline.connect(scram, 0, fec, 0);
                pipeline.connect(fec, 0, dsss, 0);
                pipeline.connect(dsss, 0, mod_, 0);
                pipeline.connect(mod_, 0, up, 0);
                pipeline.connect(up, 0, rrc, 0);
                pipeline.connect(rrc, 0, out, 0);
            }
            PipelinePreset::DmrTx => {
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 150.0));
                let crc = pipeline.add_block(BlockType::CrcGenerator { crc_type: CrcType::Crc16Ccitt }, Pos2::new(200.0, 150.0));
                let inter = pipeline.add_block(BlockType::Interleaver { rows: 8, cols: 16 }, Pos2::new(350.0, 150.0));
                let frame = pipeline.add_block(BlockType::FrameBuilder { header_bits: 48, payload_bits: 196 }, Pos2::new(500.0, 150.0));
                let gray = pipeline.add_block(BlockType::GrayMapper { bits_per_symbol: 2 }, Pos2::new(650.0, 150.0));
                let fsk = pipeline.add_block(BlockType::FskModulator { deviation_hz: 1944.0, order: 4 }, Pos2::new(800.0, 150.0));
                let up = pipeline.add_block(BlockType::Upsampler { factor: 8 }, Pos2::new(950.0, 150.0));
                let rrc = pipeline.add_block(BlockType::PulseShaper { shape: PulseShape::RootRaisedCosine, rolloff: 0.2, span: 5 }, Pos2::new(1100.0, 150.0));
                let out = pipeline.add_block(BlockType::IqOutput, Pos2::new(1250.0, 150.0));

                pipeline.connect(src, 0, crc, 0);
                pipeline.connect(crc, 0, inter, 0);
                pipeline.connect(inter, 0, frame, 0);
                pipeline.connect(frame, 0, gray, 0);
                pipeline.connect(gray, 0, fsk, 0);
                pipeline.connect(fsk, 0, up, 0);
                pipeline.connect(up, 0, rrc, 0);
                pipeline.connect(rrc, 0, out, 0);
            }
            PipelinePreset::OfdmTxRx => {
                // TX chain
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 100.0));
                let scram = pipeline.add_block(BlockType::Scrambler { polynomial: 0x48, seed: 0x7F }, Pos2::new(200.0, 100.0));
                let fec = pipeline.add_block(BlockType::FecEncoder { code_type: FecType::Convolutional, rate: "1/2".into() }, Pos2::new(350.0, 100.0));
                let inter = pipeline.add_block(BlockType::Interleaver { rows: 16, cols: 12 }, Pos2::new(500.0, 100.0));
                let ofdm_tx = pipeline.add_block(BlockType::OfdmModulator { fft_size: 64, cp_len: 16, data_carriers: 48 }, Pos2::new(650.0, 100.0));

                // Channel
                let fading = pipeline.add_block(BlockType::FadingChannel { model: FadingModel::Rayleigh, doppler_hz: 50.0 }, Pos2::new(800.0, 100.0));
                let awgn = pipeline.add_block(BlockType::AwgnChannel { snr_db: 20.0 }, Pos2::new(950.0, 100.0));
                let cfo = pipeline.add_block(BlockType::FrequencyOffset { offset_hz: 500.0 }, Pos2::new(1100.0, 100.0));

                // RX chain
                let agc = pipeline.add_block(BlockType::Agc { mode: AgcMode::Fast, target_db: -20.0 }, Pos2::new(50.0, 250.0));
                let timing = pipeline.add_block(BlockType::TimingRecovery { algorithm: TimingAlgo::EarlyLate, loop_bw: 0.02 }, Pos2::new(200.0, 250.0));
                let carrier = pipeline.add_block(BlockType::CarrierRecovery { algorithm: CarrierAlgo::PilotAided, loop_bw: 0.01 }, Pos2::new(350.0, 250.0));
                let eq = pipeline.add_block(BlockType::Equalizer { eq_type: EqualizerType::Lms, taps: 15, mu: 0.005 }, Pos2::new(500.0, 250.0));
                let out = pipeline.add_block(BlockType::BitOutput, Pos2::new(650.0, 250.0));

                pipeline.connect(src, 0, scram, 0);
                pipeline.connect(scram, 0, fec, 0);
                pipeline.connect(fec, 0, inter, 0);
                pipeline.connect(inter, 0, ofdm_tx, 0);
                pipeline.connect(ofdm_tx, 0, fading, 0);
                pipeline.connect(fading, 0, awgn, 0);
                pipeline.connect(awgn, 0, cfo, 0);
                pipeline.connect(cfo, 0, agc, 0);
                pipeline.connect(agc, 0, timing, 0);
                pipeline.connect(timing, 0, carrier, 0);
                pipeline.connect(carrier, 0, eq, 0);
                pipeline.connect(eq, 0, out, 0);
            }
            PipelinePreset::ParallelIqDemo => {
                // Demonstrates I/Q split and parallel processing
                let src = pipeline.add_block(BlockType::BitSource { pattern: "random".into() }, Pos2::new(50.0, 200.0));
                let mod_ = pipeline.add_block(BlockType::PskModulator { order: 4 }, Pos2::new(200.0, 200.0));

                // Split I and Q
                let split = pipeline.add_block(BlockType::IqSplit, Pos2::new(350.0, 200.0));

                // Process I branch
                let fir_i = pipeline.add_block(BlockType::FirFilter { filter_type: FilterType::Lowpass, cutoff_hz: 5000.0, num_taps: 32 }, Pos2::new(500.0, 100.0));

                // Process Q branch
                let fir_q = pipeline.add_block(BlockType::FirFilter { filter_type: FilterType::Lowpass, cutoff_hz: 5000.0, num_taps: 32 }, Pos2::new(500.0, 300.0));

                // Merge back
                let merge = pipeline.add_block(BlockType::IqMerge, Pos2::new(650.0, 200.0));
                let out = pipeline.add_block(BlockType::IqOutput, Pos2::new(800.0, 200.0));

                pipeline.connect(src, 0, mod_, 0);
                pipeline.connect(mod_, 0, split, 0);
                pipeline.connect(split, 0, fir_i, 0);  // I output
                pipeline.connect(split, 1, fir_q, 0);  // Q output
                pipeline.connect(fir_i, 0, merge, 0);  // I input
                pipeline.connect(fir_q, 0, merge, 1);  // Q input
                pipeline.connect(merge, 0, out, 0);
            }
        }

        pipeline
    }

    /// Validate the pipeline and return validation result
    pub fn validate(&self) -> ValidationResult {
        let mut result = ValidationResult {
            is_valid: true,
            warnings: Vec::new(),
            errors: Vec::new(),
        };

        if self.blocks.is_empty() {
            result.warnings.push("Pipeline is empty".to_string());
            return result;
        }

        // Check for source blocks
        let source_blocks: Vec<_> = self.blocks.values()
            .filter(|b| b.block_type.num_inputs() == 0)
            .collect();
        if source_blocks.is_empty() {
            result.errors.push("No source blocks found - pipeline has no input".to_string());
            result.is_valid = false;
        }

        // Check for output blocks
        let output_blocks: Vec<_> = self.blocks.values()
            .filter(|b| b.block_type.num_outputs() == 0)
            .collect();
        if output_blocks.is_empty() {
            result.errors.push("No output blocks found - pipeline has no output".to_string());
            result.is_valid = false;
        }

        // Check for unconnected inputs
        for block in self.blocks.values() {
            let num_inputs = block.block_type.num_inputs();
            if num_inputs > 0 {
                for port in 0..num_inputs {
                    let connected = self.connections.iter()
                        .any(|c| c.to_block == block.id && c.to_port == port);
                    if !connected {
                        result.warnings.push(format!(
                            "Block '{}' (ID {}) has unconnected input port {}",
                            block.name, block.id, port
                        ));
                    }
                }
            }
        }

        // Check for unconnected outputs (warning only for non-output blocks)
        for block in self.blocks.values() {
            let num_outputs = block.block_type.num_outputs();
            if num_outputs > 0 {
                for port in 0..num_outputs {
                    let connected = self.connections.iter()
                        .any(|c| c.from_block == block.id && c.from_port == port);
                    if !connected {
                        result.warnings.push(format!(
                            "Block '{}' (ID {}) has unconnected output port {}",
                            block.name, block.id, port
                        ));
                    }
                }
            }
        }

        // Check for cycles using DFS
        if self.has_cycle() {
            result.errors.push("Pipeline contains a cycle".to_string());
            result.is_valid = false;
        }

        // Check disabled blocks
        for block in self.blocks.values() {
            if !block.enabled {
                result.warnings.push(format!("Block '{}' is disabled", block.name));
            }
        }

        result
    }

    /// Check if the pipeline contains a cycle
    fn has_cycle(&self) -> bool {
        let mut visited = HashSet::new();
        let mut rec_stack = HashSet::new();

        for &id in self.blocks.keys() {
            if self.dfs_cycle_check(id, &mut visited, &mut rec_stack) {
                return true;
            }
        }
        false
    }

    fn dfs_cycle_check(&self, node: BlockId, visited: &mut HashSet<BlockId>, rec_stack: &mut HashSet<BlockId>) -> bool {
        if rec_stack.contains(&node) {
            return true;
        }
        if visited.contains(&node) {
            return false;
        }

        visited.insert(node);
        rec_stack.insert(node);

        for conn in &self.connections {
            if conn.from_block == node {
                if self.dfs_cycle_check(conn.to_block, visited, rec_stack) {
                    return true;
                }
            }
        }

        rec_stack.remove(&node);
        false
    }

    /// Check if adding a connection from `from_block` to `to_block` would create a cycle.
    /// A cycle would be created if `to_block` can already reach `from_block` through existing connections.
    pub fn would_create_cycle(&self, from_block: BlockId, to_block: BlockId) -> bool {
        // Self-loops are always cycles
        if from_block == to_block {
            return true;
        }

        // Check if to_block can reach from_block through existing connections
        // Using BFS to find if there's a path from to_block to from_block
        let mut visited = HashSet::new();
        let mut queue = vec![to_block];

        while let Some(current) = queue.pop() {
            if current == from_block {
                return true; // Found a path from to_block to from_block
            }

            if visited.contains(&current) {
                continue;
            }
            visited.insert(current);

            // Add all downstream blocks from current
            for conn in &self.connections {
                if conn.from_block == current && !visited.contains(&conn.to_block) {
                    queue.push(conn.to_block);
                }
            }
        }

        false
    }

    /// Get connected port info for a block
    pub fn get_input_connections(&self, block_id: BlockId) -> Vec<(u32, BlockId, u32)> {
        self.connections.iter()
            .filter(|c| c.to_block == block_id)
            .map(|c| (c.to_port, c.from_block, c.from_port))
            .collect()
    }

    pub fn get_output_connections(&self, block_id: BlockId) -> Vec<(u32, BlockId, u32)> {
        self.connections.iter()
            .filter(|c| c.from_block == block_id)
            .map(|c| (c.from_port, c.to_block, c.to_port))
            .collect()
    }

    /// Auto-layout the pipeline using topological sort
    pub fn auto_layout(&mut self) {
        // Default to horizontal layout for backward compatibility
        self.auto_layout_with_orientation(false);
    }

    pub fn auto_layout_with_orientation(&mut self, vertical: bool) {
        if self.blocks.is_empty() {
            return;
        }

        // Find source blocks (no inputs or no incoming connections)
        let source_blocks: Vec<BlockId> = self.blocks.iter()
            .filter(|(id, block)| {
                block.block_type.num_inputs() == 0 ||
                !self.connections.iter().any(|c| c.to_block == **id)
            })
            .map(|(id, _)| *id)
            .collect();

        if source_blocks.is_empty() {
            // If no sources, just arrange in a grid
            self.grid_layout();
            return;
        }

        // Topological sort with levels
        let mut levels: HashMap<BlockId, usize> = HashMap::new();
        let mut visited = HashSet::new();
        let mut queue: Vec<(BlockId, usize)> = source_blocks.iter().map(|id| (*id, 0)).collect();

        while let Some((block_id, level)) = queue.pop() {
            if visited.contains(&block_id) {
                continue;
            }
            visited.insert(block_id);
            levels.insert(block_id, level);

            // Add all blocks connected from this block
            for conn in &self.connections {
                if conn.from_block == block_id && !visited.contains(&conn.to_block) {
                    queue.push((conn.to_block, level + 1));
                }
            }
        }

        // Add any unvisited blocks
        for id in self.blocks.keys() {
            if !levels.contains_key(id) {
                let max_level = levels.values().max().copied().unwrap_or(0);
                levels.insert(*id, max_level + 1);
            }
        }

        // Group blocks by level
        let mut level_groups: HashMap<usize, Vec<BlockId>> = HashMap::new();
        for (id, level) in &levels {
            level_groups.entry(*level).or_default().push(*id);
        }

        // Position blocks based on orientation
        let block_width = 140.0;
        let block_height = 70.0;
        let h_spacing = 30.0;
        let v_spacing = 30.0;

        if vertical {
            // Vertical layout: levels go top-to-bottom, blocks at same level go left-to-right
            for (level, block_ids) in &level_groups {
                let num_blocks = block_ids.len();
                let total_width = num_blocks as f32 * block_width + (num_blocks - 1) as f32 * h_spacing;
                let start_x = (400.0 - total_width / 2.0).max(50.0);

                for (idx, block_id) in block_ids.iter().enumerate() {
                    if let Some(block) = self.blocks.get_mut(block_id) {
                        block.position = Pos2::new(
                            start_x + idx as f32 * (block_width + h_spacing),
                            50.0 + *level as f32 * (block_height + v_spacing),
                        );
                    }
                }
            }
        } else {
            // Horizontal layout: levels go left-to-right, blocks at same level go top-to-bottom
            for (level, block_ids) in &level_groups {
                let num_blocks = block_ids.len();
                let total_height = num_blocks as f32 * block_height + (num_blocks - 1) as f32 * v_spacing;
                let start_y = (300.0 - total_height / 2.0).max(50.0);

                for (idx, block_id) in block_ids.iter().enumerate() {
                    if let Some(block) = self.blocks.get_mut(block_id) {
                        block.position = Pos2::new(
                            50.0 + *level as f32 * (block_width + h_spacing),
                            start_y + idx as f32 * (block_height + v_spacing),
                        );
                    }
                }
            }
        }
    }

    /// Simple grid layout for disconnected blocks
    pub fn grid_layout(&mut self) {
        self.grid_layout_with_cols(4);
    }

    /// Grid layout with configurable columns, using topological order for signal flow
    pub fn grid_layout_with_cols(&mut self, cols: usize) {
        if self.blocks.is_empty() {
            return;
        }

        // Use topological sort to respect signal flow
        let sorted_ids = self.topological_sort();

        let block_width = 150.0;
        let block_height = 80.0;
        let h_spacing = 20.0;
        let v_spacing = 20.0;

        // Always left-to-right reading order
        for (idx, id) in sorted_ids.iter().enumerate() {
            let col = idx % cols;
            let row = idx / cols;
            if let Some(block) = self.blocks.get_mut(id) {
                block.position = Pos2::new(
                    50.0 + col as f32 * (block_width + h_spacing),
                    50.0 + row as f32 * (block_height + v_spacing),
                );
            }
        }
    }

    /// Topological sort of blocks based on connections (for signal flow order)
    fn topological_sort(&self) -> Vec<BlockId> {
        // Find source blocks (no inputs or no incoming connections)
        let source_blocks: Vec<BlockId> = self.blocks.iter()
            .filter(|(id, block)| {
                block.block_type.num_inputs() == 0 ||
                !self.connections.iter().any(|c| c.to_block == **id)
            })
            .map(|(id, _)| *id)
            .collect();

        let mut result = Vec::new();
        let mut visited = HashSet::new();
        let mut queue: Vec<BlockId> = source_blocks;

        while let Some(block_id) = queue.pop() {
            if visited.contains(&block_id) {
                continue;
            }
            visited.insert(block_id);
            result.push(block_id);

            // Find all blocks that this block connects to
            let mut next_blocks: Vec<BlockId> = self.connections.iter()
                .filter(|c| c.from_block == block_id && !visited.contains(&c.to_block))
                .map(|c| c.to_block)
                .collect();
            // Sort to get deterministic order
            next_blocks.sort();
            // Add in reverse so we pop in order
            for id in next_blocks.into_iter().rev() {
                queue.push(id);
            }
        }

        // Add any unvisited blocks (disconnected) at the end, sorted by ID
        let mut unvisited: Vec<BlockId> = self.blocks.keys()
            .filter(|id| !visited.contains(id))
            .cloned()
            .collect();
        unvisited.sort();
        result.extend(unvisited);

        result
    }

    /// Compact layout - minimize spacing while respecting signal flow
    pub fn compact_layout(&mut self, vertical: bool) {
        if self.blocks.is_empty() {
            return;
        }

        // Use topological layout with tighter spacing
        let block_width = 130.0;
        let block_height = 60.0;
        let h_spacing = 15.0;
        let v_spacing = 15.0;

        // Find source blocks
        let source_blocks: Vec<BlockId> = self.blocks.iter()
            .filter(|(id, block)| {
                block.block_type.num_inputs() == 0 ||
                !self.connections.iter().any(|c| c.to_block == **id)
            })
            .map(|(id, _)| *id)
            .collect();

        if source_blocks.is_empty() {
            self.grid_layout_with_cols(6);
            return;
        }

        // Topological sort with levels
        let mut levels: HashMap<BlockId, usize> = HashMap::new();
        let mut visited = HashSet::new();
        let mut queue: Vec<(BlockId, usize)> = source_blocks.iter().map(|id| (*id, 0)).collect();

        while let Some((block_id, level)) = queue.pop() {
            if visited.contains(&block_id) {
                continue;
            }
            visited.insert(block_id);
            levels.insert(block_id, level);

            for conn in &self.connections {
                if conn.from_block == block_id && !visited.contains(&conn.to_block) {
                    queue.push((conn.to_block, level + 1));
                }
            }
        }

        // Add unvisited blocks
        for id in self.blocks.keys() {
            if !levels.contains_key(id) {
                let max_level = levels.values().max().copied().unwrap_or(0);
                levels.insert(*id, max_level + 1);
            }
        }

        // Group by level
        let mut level_groups: HashMap<usize, Vec<BlockId>> = HashMap::new();
        for (id, level) in &levels {
            level_groups.entry(*level).or_default().push(*id);
        }

        // Position with compact spacing
        if vertical {
            for (level, block_ids) in &level_groups {
                for (idx, block_id) in block_ids.iter().enumerate() {
                    if let Some(block) = self.blocks.get_mut(block_id) {
                        block.position = Pos2::new(
                            30.0 + idx as f32 * (block_width + h_spacing),
                            30.0 + *level as f32 * (block_height + v_spacing),
                        );
                    }
                }
            }
        } else {
            for (level, block_ids) in &level_groups {
                for (idx, block_id) in block_ids.iter().enumerate() {
                    if let Some(block) = self.blocks.get_mut(block_id) {
                        block.position = Pos2::new(
                            30.0 + *level as f32 * (block_width + h_spacing),
                            30.0 + idx as f32 * (block_height + v_spacing),
                        );
                    }
                }
            }
        }
    }

    /// Get bounding box of all blocks
    pub fn get_bounds(&self) -> Option<Rect> {
        if self.blocks.is_empty() {
            return None;
        }

        let block_width = 140.0;
        let block_height = 60.0;

        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for block in self.blocks.values() {
            min_x = min_x.min(block.position.x);
            min_y = min_y.min(block.position.y);
            max_x = max_x.max(block.position.x + block_width);
            max_y = max_y.max(block.position.y + block_height);
        }

        Some(Rect::from_min_max(
            Pos2::new(min_x, min_y),
            Pos2::new(max_x, max_y),
        ))
    }

    /// Duplicate a block
    pub fn duplicate_block(&mut self, block_id: BlockId) -> Option<BlockId> {
        // Clone the necessary data first to avoid borrow conflicts
        let (new_pos, new_type, orig_name) = {
            let block = self.blocks.get(&block_id)?;
            (
                block.position + Vec2::new(20.0, 20.0),
                block.block_type.clone(),
                block.name.clone(),
            )
        };

        let new_id = self.add_block(new_type, new_pos);
        if let Some(new_block) = self.blocks.get_mut(&new_id) {
            new_block.name = format!("{} (copy)", orig_name);
        }
        Some(new_id)
    }
}

/// Block templates available in the library
pub fn get_block_templates() -> Vec<(BlockCategory, Vec<BlockType>)> {
    vec![
        (BlockCategory::Source, vec![
            BlockType::BitSource { pattern: "random".to_string() },
            BlockType::SymbolSource { alphabet_size: 4 },
            BlockType::FileSource { path: String::new() },
        ]),
        (BlockCategory::Coding, vec![
            BlockType::Scrambler { polynomial: 0x48, seed: 0xFF },
            BlockType::FecEncoder { code_type: FecType::Convolutional, rate: "1/2".to_string() },
            BlockType::Interleaver { rows: 8, cols: 16 },
            BlockType::CrcGenerator { crc_type: CrcType::Crc16Ccitt },
        ]),
        (BlockCategory::Mapping, vec![
            BlockType::GrayMapper { bits_per_symbol: 2 },
            BlockType::ConstellationMapper { constellation: ConstellationType::Qpsk },
            BlockType::DifferentialEncoder,
        ]),
        (BlockCategory::Modulation, vec![
            BlockType::PskModulator { order: 4 },
            BlockType::QamModulator { order: 16 },
            BlockType::FskModulator { deviation_hz: 1000.0, order: 2 },
            BlockType::OfdmModulator { fft_size: 64, cp_len: 16, data_carriers: 48 },
            BlockType::DsssSpread { chips_per_symbol: 127, code_type: "Gold".to_string() },
            BlockType::FhssHop { num_channels: 50, hop_rate: 100.0 },
            BlockType::CssModulator { sf: 7, bw_hz: 125000 },
        ]),
        (BlockCategory::Filtering, vec![
            BlockType::FirFilter { filter_type: FilterType::Lowpass, cutoff_hz: 10000.0, num_taps: 64 },
            BlockType::IirFilter { design: IirDesign::Butterworth, cutoff_hz: 10000.0, order: 4 },
            BlockType::PulseShaper { shape: PulseShape::RootRaisedCosine, rolloff: 0.35, span: 8 },
            BlockType::MatchedFilter,
        ]),
        (BlockCategory::RateConversion, vec![
            BlockType::Upsampler { factor: 4 },
            BlockType::Downsampler { factor: 4 },
            BlockType::RationalResampler { up: 3, down: 2 },
            BlockType::PolyphaseResampler { up: 4, down: 1, taps_per_phase: 16 },
        ]),
        (BlockCategory::Synchronization, vec![
            BlockType::PreambleInsert { pattern: "alternating".to_string(), length: 32 },
            BlockType::SyncWordInsert { word: "0x7E".to_string() },
            BlockType::FrameBuilder { header_bits: 32, payload_bits: 256 },
            BlockType::TdmaFramer { slots: 4, slot_ms: 10.0 },
        ]),
        (BlockCategory::Impairments, vec![
            BlockType::AwgnChannel { snr_db: 20.0 },
            BlockType::FadingChannel { model: FadingModel::Rayleigh, doppler_hz: 10.0 },
            BlockType::FrequencyOffset { offset_hz: 100.0 },
            BlockType::IqImbalance { gain_db: 0.5, phase_deg: 2.0 },
        ]),
        (BlockCategory::Recovery, vec![
            BlockType::Agc { mode: AgcMode::Adaptive, target_db: -20.0 },
            BlockType::TimingRecovery { algorithm: TimingAlgo::Gardner, loop_bw: 0.01 },
            BlockType::CarrierRecovery { algorithm: CarrierAlgo::CostasLoop, loop_bw: 0.005 },
            BlockType::PskDemodulator { order: 2 },
            BlockType::QamDemodulator { order: 16 },
            BlockType::FskDemodulator { order: 2 },
            BlockType::Equalizer { eq_type: EqualizerType::Lms, taps: 11, mu: 0.01 },
        ]),
        (BlockCategory::Output, vec![
            BlockType::IqOutput,
            BlockType::BitOutput,
            BlockType::FileOutput { path: "output.iq".to_string(), format: OutputFormat::ComplexFloat32 },
            BlockType::Split { num_outputs: 2 },
            BlockType::Merge { num_inputs: 2 },
            BlockType::IqSplit,
            BlockType::IqMerge,
        ]),
    ]
}

/// Pipeline wizard view state
pub struct PipelineWizardView {
    pub pipeline: Pipeline,
    pub selected_blocks: HashSet<BlockId>,  // Multiple block selection
    pub selected_connection: Option<usize>,
    pub dragging_block: Option<BlockType>,
    pub connecting_from: Option<(BlockId, u32)>,
    pub connection_drag_pos: Option<Pos2>,
    pub canvas_offset: Vec2,
    pub zoom: f32,
    pub show_library: bool,
    pub show_properties: bool,
    pub show_presets: bool,
    pub show_validation: bool,
    pub yaml_output: String,
    pub validation_result: ValidationResult,
    pub snap_to_grid: bool,
    pub grid_size: f32,
    pub vertical_layout: bool,
    pub auto_connect: bool,
    pub last_added_block: Option<BlockId>,
    pub wants_exit: bool,
    pub show_load_menu: bool,
    pub available_specs: Vec<(String, std::path::PathBuf)>,  // (name, path)
    pub cascade_drag: bool,  // When true, dragging a block also drags all downstream blocks
    pub status_message: Option<(String, std::time::Instant)>,  // (message, when) for temporary notifications
    pub connection_style: ConnectionStyle,  // Visual routing style for connections
    pub show_arrowheads: bool,  // Show direction arrows on connections
    // Multi-selection state
    pub selection_mode: SelectionMode,
    pub selection_start: Option<Pos2>,  // Start point of rectangle/lasso
    pub lasso_points: Vec<Pos2>,        // Points for lasso selection
    // Block drag state (for accurate tracking)
    pub drag_start_pointer: Option<Pos2>,  // Screen position where drag started
    pub drag_initial_positions: HashMap<BlockId, Pos2>,  // Initial canvas positions of dragged blocks
}

impl Default for PipelineWizardView {
    fn default() -> Self {
        Self {
            pipeline: Pipeline::new(),
            selected_blocks: HashSet::new(),
            selected_connection: None,
            dragging_block: None,
            connecting_from: None,
            connection_drag_pos: None,
            canvas_offset: Vec2::ZERO,
            zoom: 1.0,
            show_library: true,
            show_properties: true,
            show_presets: false,
            show_validation: false,
            yaml_output: String::new(),
            validation_result: ValidationResult::default(),
            snap_to_grid: true,
            grid_size: 20.0,
            vertical_layout: true,
            auto_connect: true,
            last_added_block: None,
            wants_exit: false,
            show_load_menu: false,
            available_specs: Vec::new(),
            cascade_drag: true,  // Default to cascading drag
            status_message: None,
            connection_style: ConnectionStyle::default(),
            show_arrowheads: true,  // Show direction arrows by default
            selection_mode: SelectionMode::None,
            selection_start: None,
            lasso_points: Vec::new(),
            drag_start_pointer: None,
            drag_initial_positions: HashMap::new(),
        }
    }
}

impl PipelineWizardView {
    /// Get single selected block (if exactly one is selected)
    fn single_selected_block(&self) -> Option<BlockId> {
        if self.selected_blocks.len() == 1 {
            self.selected_blocks.iter().next().copied()
        } else {
            None
        }
    }

    /// Check if a block is selected
    fn is_block_selected(&self, id: BlockId) -> bool {
        self.selected_blocks.contains(&id)
    }

    /// Check if an input port has a horizontal connection (from the left side)
    fn is_input_connected_horizontal(&self, block_id: BlockId, port: u32, canvas_rect: Rect) -> bool {
        self.pipeline.connections.iter().any(|c| {
            if c.to_block == block_id && c.to_port == port {
                // Check if this connection is horizontal
                if let (Some(from_block), Some(to_block)) = (
                    self.pipeline.blocks.get(&c.from_block),
                    self.pipeline.blocks.get(&c.to_block),
                ) {
                    return !self.is_vertical_connection(from_block, to_block, canvas_rect);
                }
            }
            false
        })
    }

    /// Check if an input port has a vertical connection (from the top side)
    fn is_input_connected_vertical(&self, block_id: BlockId, port: u32, canvas_rect: Rect) -> bool {
        self.pipeline.connections.iter().any(|c| {
            if c.to_block == block_id && c.to_port == port {
                // Check if this connection is vertical
                if let (Some(from_block), Some(to_block)) = (
                    self.pipeline.blocks.get(&c.from_block),
                    self.pipeline.blocks.get(&c.to_block),
                ) {
                    return self.is_vertical_connection(from_block, to_block, canvas_rect);
                }
            }
            false
        })
    }

    /// Check if an output port has a horizontal connection (from the right side)
    fn is_output_connected_horizontal(&self, block_id: BlockId, port: u32, canvas_rect: Rect) -> bool {
        self.pipeline.connections.iter().any(|c| {
            if c.from_block == block_id && c.from_port == port {
                // Check if this connection is horizontal
                if let (Some(from_block), Some(to_block)) = (
                    self.pipeline.blocks.get(&c.from_block),
                    self.pipeline.blocks.get(&c.to_block),
                ) {
                    return !self.is_vertical_connection(from_block, to_block, canvas_rect);
                }
            }
            false
        })
    }

    /// Check if an output port has a vertical connection (from the bottom side)
    fn is_output_connected_vertical(&self, block_id: BlockId, port: u32, canvas_rect: Rect) -> bool {
        self.pipeline.connections.iter().any(|c| {
            if c.from_block == block_id && c.from_port == port {
                // Check if this connection is vertical
                if let (Some(from_block), Some(to_block)) = (
                    self.pipeline.blocks.get(&c.from_block),
                    self.pipeline.blocks.get(&c.to_block),
                ) {
                    return self.is_vertical_connection(from_block, to_block, canvas_rect);
                }
            }
            false
        })
    }

    /// Select a single block (clearing any previous selection)
    fn select_single_block(&mut self, id: BlockId) {
        self.selected_blocks.clear();
        self.selected_blocks.insert(id);
        self.selected_connection = None;
    }

    /// Toggle block selection (for shift+click)
    fn toggle_block_selection(&mut self, id: BlockId) {
        if self.selected_blocks.contains(&id) {
            self.selected_blocks.remove(&id);
        } else {
            self.selected_blocks.insert(id);
        }
        self.selected_connection = None;
    }

    /// Clear all selection
    fn clear_selection(&mut self) {
        self.selected_blocks.clear();
        self.selected_connection = None;
        self.selection_mode = SelectionMode::None;
        self.selection_start = None;
        self.lasso_points.clear();
    }

    /// Fit all blocks to the visible canvas by adjusting zoom and offset
    fn fit_to_view(&mut self, canvas_size: Vec2) {
        if let Some(bounds) = self.pipeline.get_bounds() {
            let padding = 50.0;
            let content_width = bounds.width() + padding * 2.0;
            let content_height = bounds.height() + padding * 2.0;

            // Calculate zoom to fit
            let zoom_x = canvas_size.x / content_width;
            let zoom_y = canvas_size.y / content_height;
            self.zoom = zoom_x.min(zoom_y).clamp(0.3, 2.0);

            // Center the content
            let scaled_width = content_width * self.zoom;
            let scaled_height = content_height * self.zoom;

            self.canvas_offset = Vec2::new(
                (canvas_size.x - scaled_width) / 2.0 - (bounds.min.x - padding) * self.zoom,
                (canvas_size.y - scaled_height) / 2.0 - (bounds.min.y - padding) * self.zoom,
            );
        }
    }

    /// Setup after loading a new pipeline (clear selections, apply layout)
    fn setup_after_load(&mut self) {
        self.selected_blocks.clear();
        self.selected_connection = None;
        self.last_added_block = None;
        // Apply the pipeline's saved layout mode (defaults to Grid for specs without it)
        let canvas_size = Vec2::new(800.0, 600.0);
        let mode = self.pipeline.layout_mode;
        self.apply_layout(mode, canvas_size);
    }

    /// Apply a layout mode
    fn apply_layout(&mut self, mode: LayoutMode, canvas_size: Vec2) {
        match mode {
            LayoutMode::Flow => {
                self.pipeline.auto_layout_with_orientation(self.vertical_layout);
            }
            LayoutMode::FitToView => {
                // First do flow layout, then fit to view
                self.pipeline.auto_layout_with_orientation(self.vertical_layout);
                self.fit_to_view(canvas_size);
            }
            LayoutMode::Compact => {
                self.pipeline.compact_layout(self.vertical_layout);
            }
            LayoutMode::Grid => {
                self.pipeline.grid_layout();
            }
        }
    }

    /// Check if point is inside a polygon (for lasso selection)
    fn point_in_polygon(point: Pos2, polygon: &[Pos2]) -> bool {
        if polygon.len() < 3 {
            return false;
        }
        let mut inside = false;
        let n = polygon.len();
        let mut j = n - 1;
        for i in 0..n {
            let pi = polygon[i];
            let pj = polygon[j];
            if ((pi.y > point.y) != (pj.y > point.y))
                && (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)
            {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    /// Find all blocks inside a rectangle
    fn blocks_in_rect(&self, rect: Rect, canvas_rect: Rect) -> HashSet<BlockId> {
        let mut result = HashSet::new();
        for (id, block) in &self.pipeline.blocks {
            let block_rect = self.block_rect(block, canvas_rect);
            if rect.intersects(block_rect) {
                result.insert(*id);
            }
        }
        result
    }

    /// Find all blocks inside a lasso polygon
    fn blocks_in_lasso(&self, polygon: &[Pos2], canvas_rect: Rect) -> HashSet<BlockId> {
        let mut result = HashSet::new();
        for (id, block) in &self.pipeline.blocks {
            let block_rect = self.block_rect(block, canvas_rect);
            // Check if block center is inside polygon
            if Self::point_in_polygon(block_rect.center(), polygon) {
                result.insert(*id);
            }
        }
        result
    }

    /// Find all blocks downstream from a given block (following output connections)
    fn get_downstream_blocks(&self, start_id: BlockId) -> HashSet<BlockId> {
        let mut downstream = HashSet::new();
        let mut to_visit = vec![start_id];

        while let Some(current) = to_visit.pop() {
            // Find all blocks that this block connects to
            for conn in &self.pipeline.connections {
                if conn.from_block == current && !downstream.contains(&conn.to_block) {
                    downstream.insert(conn.to_block);
                    to_visit.push(conn.to_block);
                }
            }
        }

        downstream
    }

    /// Scan the specs directory for available waveform YAML files
    #[cfg(not(target_arch = "wasm32"))]
    fn refresh_available_specs(&mut self) {
        self.available_specs.clear();

        // Look for specs in multiple locations
        let spec_dirs = [
            std::path::PathBuf::from("specs"),
            std::path::PathBuf::from("../specs"),
            std::env::current_exe()
                .ok()
                .and_then(|p| p.parent().map(|p| p.join("specs")))
                .unwrap_or_default(),
        ];

        for dir in &spec_dirs {
            if dir.exists() && dir.is_dir() {
                if let Ok(entries) = std::fs::read_dir(dir) {
                    for entry in entries.flatten() {
                        let path = entry.path();
                        if path.extension().map(|e| e == "yaml" || e == "yml").unwrap_or(false) {
                            let name = path.file_stem()
                                .and_then(|s| s.to_str())
                                .unwrap_or("unknown")
                                .to_uppercase();
                            // Avoid duplicates
                            if !self.available_specs.iter().any(|(n, _)| n == &name) {
                                self.available_specs.push((name, path));
                            }
                        }
                    }
                }
                break; // Use first found specs directory
            }
        }

        // Sort alphabetically
        self.available_specs.sort_by(|a, b| a.0.cmp(&b.0));
    }

    #[cfg(target_arch = "wasm32")]
    fn refresh_available_specs(&mut self) {
        // WASM doesn't have filesystem access
        self.available_specs.clear();
    }

    pub fn new() -> Self {
        Self::default()
    }

    pub fn render(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();

        // Top panel: Toolbar (rendered first so side panels appear below it)
        egui::TopBottomPanel::top("pipeline_toolbar")
            .show(&ctx, |ui| {
                ui.horizontal(|ui| {
                    // Exit button to return to main view
                    if ui.button("← Exit").clicked() {
                        self.wants_exit = true;
                    }
                    ui.separator();
                    ui.heading("Pipeline Waveform Builder");
                    ui.add_space(20.0);
                    ui.checkbox(&mut self.show_library, "Library");
                    ui.checkbox(&mut self.show_properties, "Properties");
                    ui.checkbox(&mut self.snap_to_grid, "Snap");
                    ui.separator();
                    // Layout orientation toggle
                    let layout_label = if self.vertical_layout { "↓ Vertical" } else { "→ Horizontal" };
                    if ui.selectable_label(false, layout_label).clicked() {
                        self.vertical_layout = !self.vertical_layout;
                    }
                    ui.checkbox(&mut self.auto_connect, "Auto-connect");
                    ui.checkbox(&mut self.cascade_drag, "Cascade drag")
                        .on_hover_text("Drag a block to also move all downstream connected blocks");

                    // Connection style dropdown
                    egui::ComboBox::from_id_salt("conn_style")
                        .selected_text(self.connection_style.name())
                        .width(90.0)
                        .show_ui(ui, |ui| {
                            for style in ConnectionStyle::all() {
                                ui.selectable_value(&mut self.connection_style, *style, style.name());
                            }
                        })
                        .response.on_hover_text("Connection line routing style");

                    ui.checkbox(&mut self.show_arrowheads, "Arrows")
                        .on_hover_text("Show arrowheads on connections");

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("Export YAML").clicked() {
                            self.yaml_output = self.pipeline.to_yaml();
                        }
                        // Save pipeline to file
                        if ui.button("Save").clicked() {
                            #[cfg(not(target_arch = "wasm32"))]
                            if let Some(path) = rfd::FileDialog::new()
                                .set_file_name(&format!("{}.yaml", self.pipeline.name.replace(' ', "_").to_lowercase()))
                                .add_filter("YAML", &["yaml", "yml"])
                                .save_file()
                            {
                                let yaml = self.pipeline.to_yaml_structured();
                                if let Err(e) = std::fs::write(&path, &yaml) {
                                    log::error!("Failed to save pipeline: {}", e);
                                }
                            }
                        }
                        // Load pipeline dropdown menu
                        egui::menu::menu_button(ui, "Load ▼", |ui| {
                            // Refresh specs list when menu opens
                            if self.available_specs.is_empty() {
                                self.refresh_available_specs();
                            }

                            ui.set_min_width(200.0);

                            // List available specs with TX/RX/Loopback submenus
                            if !self.available_specs.is_empty() {
                                ui.label(RichText::new("Available Specs").strong());
                                ui.separator();

                                // Clone the specs to avoid borrow issues
                                let specs: Vec<_> = self.available_specs.clone();
                                for (name, path) in specs {
                                    #[cfg(not(target_arch = "wasm32"))]
                                    {
                                        // Read file to detect sections
                                        if let Ok(yaml) = std::fs::read_to_string(&path) {
                                            let (has_tx, has_rx, has_channel, has_legacy) = Pipeline::detect_spec_sections(&yaml);

                                            if has_tx || has_rx {
                                                // Show submenu with TX/RX/Loopback options
                                                ui.menu_button(&name, |ui| {
                                                    if has_tx {
                                                        if ui.button("📤 TX Pipeline").clicked() {
                                                            match Pipeline::from_yaml_with_mode(&yaml, LoadMode::TxOnly) {
                                                                Ok(pipeline) => {
                                                                    self.pipeline = pipeline;
                                                                    self.setup_after_load();
                                                                }
                                                                Err(e) => log::error!("Failed to load TX: {}", e),
                                                            }
                                                            ui.close_menu();
                                                        }
                                                    }
                                                    if has_rx {
                                                        if ui.button("📥 RX Pipeline").clicked() {
                                                            match Pipeline::from_yaml_with_mode(&yaml, LoadMode::RxOnly) {
                                                                Ok(pipeline) => {
                                                                    self.pipeline = pipeline;
                                                                    self.setup_after_load();
                                                                }
                                                                Err(e) => log::error!("Failed to load RX: {}", e),
                                                            }
                                                            ui.close_menu();
                                                        }
                                                    }
                                                    if has_tx && has_rx {
                                                        if ui.button("🔄 Loopback (TX→RX)").clicked() {
                                                            match Pipeline::from_yaml_with_mode(&yaml, LoadMode::Loopback) {
                                                                Ok(pipeline) => {
                                                                    self.pipeline = pipeline;
                                                                    self.setup_after_load();
                                                                }
                                                                Err(e) => log::error!("Failed to load Loopback: {}", e),
                                                            }
                                                            ui.close_menu();
                                                        }
                                                    }
                                                    if has_channel {
                                                        ui.separator();
                                                        if ui.button("📡 Channel Only").clicked() {
                                                            match Pipeline::from_yaml_with_mode(&yaml, LoadMode::ChannelOnly) {
                                                                Ok(pipeline) => {
                                                                    self.pipeline = pipeline;
                                                                    self.setup_after_load();
                                                                }
                                                                Err(e) => log::error!("Failed to load Channel: {}", e),
                                                            }
                                                            ui.close_menu();
                                                        }
                                                    }
                                                });
                                            } else if has_legacy {
                                                // Legacy format - single button
                                                if ui.button(&name).clicked() {
                                                    match Pipeline::from_yaml(&yaml) {
                                                        Ok(pipeline) => {
                                                            self.pipeline = pipeline;
                                                            self.setup_after_load();
                                                        }
                                                        Err(e) => log::error!("Failed to parse {}: {}", name, e),
                                                    }
                                                    ui.close_menu();
                                                }
                                            }
                                        }
                                    }
                                }
                                ui.separator();
                            }

                            // Import from file option
                            if ui.button("📁 Import from file...").clicked() {
                                #[cfg(not(target_arch = "wasm32"))]
                                if let Some(path) = rfd::FileDialog::new()
                                    .add_filter("YAML", &["yaml", "yml"])
                                    .pick_file()
                                {
                                    match std::fs::read_to_string(&path) {
                                        Ok(yaml) => {
                                            match Pipeline::from_yaml(&yaml) {
                                                Ok(pipeline) => {
                                                    self.pipeline = pipeline;
                                                    self.setup_after_load();
                                                }
                                                Err(e) => log::error!("Failed to parse pipeline: {}", e),
                                            }
                                        }
                                        Err(e) => log::error!("Failed to read file: {}", e),
                                    }
                                }
                                ui.close_menu();
                            }

                            // Refresh option
                            ui.separator();
                            if ui.button("🔄 Refresh list").clicked() {
                                self.refresh_available_specs();
                            }
                        });
                        ui.separator();
                        if ui.button("Validate").clicked() {
                            self.validation_result = self.pipeline.validate();
                            self.show_validation = true;
                        }
                        // Layout dropdown menu - show current layout mode
                        let current_layout = self.pipeline.layout_mode;
                        egui::menu::menu_button(ui, format!("Layout: {} ▼", current_layout.name()), |ui| {
                            ui.set_min_width(150.0);
                            for mode in LayoutMode::all() {
                                let is_selected = *mode == current_layout;
                                let label = if is_selected { format!("✓ {}", mode.name()) } else { mode.name().to_string() };
                                if ui.button(label).on_hover_text(mode.description()).clicked() {
                                    // Save layout mode to pipeline
                                    self.pipeline.layout_mode = *mode;
                                    // Use a reasonable default canvas size for fit-to-view
                                    let canvas_size = Vec2::new(800.0, 600.0);
                                    self.apply_layout(*mode, canvas_size);
                                    ui.close_menu();
                                }
                            }
                            ui.separator();
                            if ui.button("Reset Zoom").on_hover_text("Reset zoom to 100%").clicked() {
                                self.zoom = 1.0;
                                ui.close_menu();
                            }
                            if ui.button("Center View").on_hover_text("Center blocks in view").clicked() {
                                if let Some(bounds) = self.pipeline.get_bounds() {
                                    self.canvas_offset = Vec2::new(
                                        100.0 - bounds.min.x * self.zoom,
                                        100.0 - bounds.min.y * self.zoom,
                                    );
                                }
                                ui.close_menu();
                            }
                        });
                        if ui.button("Presets").clicked() {
                            self.show_presets = !self.show_presets;
                        }
                        if ui.button("Clear").clicked() {
                            self.pipeline = Pipeline::new();
                            self.selected_blocks.clear();
                            self.selected_connection = None;
                            self.connecting_from = None;
                            self.last_added_block = None;
                        }
                    });
                });
            });

        // Preset selection window
        if self.show_presets {
            egui::Window::new("Pipeline Presets")
                .resizable(false)
                .collapsible(false)
                .show(&ctx, |ui| {
                    ui.label("Load a preset pipeline template:");
                    ui.add_space(5.0);
                    for preset in PipelinePreset::all() {
                        ui.horizontal(|ui| {
                            if ui.button(preset.name()).clicked() {
                                self.pipeline = Pipeline::from_preset(*preset);
                                self.selected_blocks.clear();
                                self.selected_connection = None;
                                self.last_added_block = None;
                                self.show_presets = false;
                            }
                            ui.label(RichText::new(preset.description()).italics().weak());
                        });
                    }
                    ui.add_space(5.0);
                    if ui.button("Close").clicked() {
                        self.show_presets = false;
                    }
                });
        }

        // Validation window
        if self.show_validation {
            egui::Window::new("Pipeline Validation")
                .resizable(true)
                .collapsible(false)
                .show(&ctx, |ui| {
                    if self.validation_result.is_valid && self.validation_result.warnings.is_empty() {
                        ui.colored_label(Color32::GREEN, "Pipeline is valid!");
                    } else if self.validation_result.is_valid {
                        ui.colored_label(Color32::YELLOW, "Pipeline is valid with warnings");
                    } else {
                        ui.colored_label(Color32::RED, "Pipeline has errors");
                    }

                    if !self.validation_result.errors.is_empty() {
                        ui.add_space(5.0);
                        ui.label(RichText::new("Errors:").strong().color(Color32::RED));
                        for error in &self.validation_result.errors {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("  ").color(Color32::RED));
                                ui.label(error);
                            });
                        }
                    }

                    if !self.validation_result.warnings.is_empty() {
                        ui.add_space(5.0);
                        ui.label(RichText::new("Warnings:").strong().color(Color32::YELLOW));
                        for warning in &self.validation_result.warnings {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("  ").color(Color32::YELLOW));
                                ui.label(warning);
                            });
                        }
                    }

                    ui.add_space(10.0);
                    if ui.button("Close").clicked() {
                        self.show_validation = false;
                    }
                });
        }

        // Left panel: Block Library (uses cloned ctx so it respects TopBottomPanel)
        if self.show_library {
            egui::SidePanel::left("pipeline_library_panel")
                .resizable(true)
                .default_width(200.0)
                .min_width(150.0)
                .max_width(350.0)
                .show(&ctx, |ui| {
                    ui.heading("Block Library");
                    ui.separator();
                    self.render_library_content(ui);
                });
        }

        // Right panel: Properties (uses cloned ctx so it respects TopBottomPanel)
        if self.show_properties {
            egui::SidePanel::right("pipeline_properties_panel")
                .resizable(true)
                .default_width(250.0)
                .min_width(200.0)
                .max_width(400.0)
                .show(&ctx, |ui| {
                    ui.heading("Properties");
                    ui.separator();
                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            self.render_properties_content(ui);
                        });
                });
        }

        // Bottom status bar for temporary messages
        if self.status_message.is_some() {
            egui::TopBottomPanel::bottom("status_bar").show(&ctx, |ui| {
                ui.horizontal(|ui| {
                    // Check if message has expired (3 seconds)
                    let should_clear = if let Some((ref msg, instant)) = self.status_message {
                        let elapsed = instant.elapsed().as_secs_f32();
                        if elapsed > 3.0 {
                            true
                        } else {
                            // Show warning icon and message
                            ui.label(RichText::new("⚠").color(Color32::YELLOW));
                            ui.label(RichText::new(msg).color(Color32::YELLOW));
                            false
                        }
                    } else {
                        false
                    };

                    if should_clear {
                        // Will be cleared after the borrow ends
                    }
                });
            });

            // Clear expired messages
            if let Some((_, instant)) = &self.status_message {
                if instant.elapsed().as_secs_f32() > 3.0 {
                    self.status_message = None;
                }
            }
        }

        // Main canvas takes the remaining central area
        egui::CentralPanel::default().show(&ctx, |ui| {
            self.render_canvas(ui);
        });

        // YAML output window
        if !self.yaml_output.is_empty() {
            egui::Window::new("Pipeline YAML")
                .resizable(true)
                .default_width(500.0)
                .show(&ctx, |ui| {
                    ui.horizontal(|ui| {
                        if ui.button("Copy").clicked() {
                            ui.output_mut(|o| o.copied_text = self.yaml_output.clone());
                        }
                        if ui.button("Close").clicked() {
                            self.yaml_output.clear();
                        }
                    });
                    ui.separator();
                    egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
                        ui.add(egui::TextEdit::multiline(&mut self.yaml_output.as_str())
                            .font(egui::TextStyle::Monospace)
                            .desired_width(f32::INFINITY));
                    });
                });
        }
    }

    fn render_library_content(&mut self, ui: &mut Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            for (category, templates) in get_block_templates() {
                // Use CollapsingHeader with default_open for first few categories
                let default_open = matches!(category,
                    BlockCategory::Source | BlockCategory::Modulation | BlockCategory::Filtering);

                egui::CollapsingHeader::new(RichText::new(category.name()).color(category.color()))
                    .default_open(default_open)
                    .show(ui, |ui| {
                        for template in templates {
                            let response = ui.add(
                                egui::Button::new(template.name())
                                    .fill(category.color().gamma_multiply(0.3))
                                    .min_size(egui::vec2(170.0, 22.0))
                            );

                            if response.clicked() {
                                // Find auto-connect source before adding new block
                                let auto_connect_source = if self.auto_connect && template.num_inputs() > 0 {
                                    self.pipeline.find_auto_connect_source(self.vertical_layout)
                                } else {
                                    None
                                };

                                // Add block at an empty position based on layout orientation
                                let pos = self.pipeline.find_empty_position(self.vertical_layout);
                                let new_block_id = self.pipeline.add_block(template.clone(), pos);
                                self.last_added_block = Some(new_block_id);

                                // Auto-connect if enabled and we found a source
                                if let Some(source_id) = auto_connect_source {
                                    if let Some(from_port) = self.pipeline.find_free_output_port(source_id) {
                                        // Connect first free output to first input of new block
                                        self.pipeline.connect(source_id, from_port, new_block_id, 0);
                                    }
                                }
                            }

                            response.on_hover_text(format!("Click to add {}", template.name()));
                        }
                    });
            }
        });
    }

    fn render_canvas(&mut self, ui: &mut Ui) {
        let (response, painter) = ui.allocate_painter(
            ui.available_size(),
            egui::Sense::click_and_drag(),
        );

        let rect = response.rect;
        let pointer_pos = response.hover_pos();

        // Background
        painter.rect_filled(rect, 0.0, Color32::from_rgb(30, 30, 40));

        // Grid
        let grid_spacing = self.grid_size * self.zoom;
        let offset = self.canvas_offset;

        for x in (0..((rect.width() / grid_spacing) as i32 + 1)).map(|i| rect.left() + i as f32 * grid_spacing + offset.x % grid_spacing) {
            painter.line_segment(
                [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
                Stroke::new(1.0, Color32::from_rgb(50, 50, 60)),
            );
        }
        for y in (0..((rect.height() / grid_spacing) as i32 + 1)).map(|i| rect.top() + i as f32 * grid_spacing + offset.y % grid_spacing) {
            painter.line_segment(
                [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
                Stroke::new(1.0, Color32::from_rgb(50, 50, 60)),
            );
        }

        // Draw connections with adaptive port positioning
        for (idx, conn) in self.pipeline.connections.iter().enumerate() {
            if let (Some(from_block), Some(to_block)) = (
                self.pipeline.blocks.get(&conn.from_block),
                self.pipeline.blocks.get(&conn.to_block),
            ) {
                let is_vertical = self.is_vertical_connection(from_block, to_block, rect);
                let from_pos = self.block_output_pos_for_connection(from_block, conn.from_port, to_block, rect);
                let to_pos = self.block_input_pos_for_connection(to_block, conn.to_port, from_block, rect);

                let is_selected = self.selected_connection == Some(idx);
                let color = if is_selected {
                    Color32::from_rgb(255, 200, 100)
                } else {
                    Color32::from_rgb(100, 200, 100)
                };
                let width = if is_selected { 3.0 } else { 2.0 };

                self.draw_connection(&painter, from_pos, to_pos, is_vertical, color, width);
            }
        }

        // Draw in-progress connection
        if let Some((from_block_id, from_port)) = self.connecting_from {
            if let Some(from_block) = self.pipeline.blocks.get(&from_block_id) {
                let from_pos = self.block_output_pos(from_block, from_port, rect);
                let to_pos = pointer_pos.unwrap_or(from_pos);

                // Check if hovering near an input port and if it would create a cycle
                let mut would_cycle = false;
                let mut hover_block_name = None;
                if let Some(cursor_pos) = pointer_pos {
                    for (id, block) in &self.pipeline.blocks {
                        let block_rect = self.block_rect(block, rect);
                        let num_inputs = block.block_type.num_inputs();
                        for port in 0..num_inputs {
                            let pos_left = self.block_input_pos(block, port, rect);
                            let spacing = block_rect.width() / (num_inputs + 1) as f32;
                            let pos_top = Pos2::new(block_rect.left() + spacing * (port + 1) as f32, block_rect.top());

                            if (cursor_pos - pos_left).length() < 15.0 * self.zoom
                               || (cursor_pos - pos_top).length() < 15.0 * self.zoom {
                                if self.pipeline.would_create_cycle(from_block_id, *id) {
                                    would_cycle = true;
                                    hover_block_name = Some(block.name.clone());
                                }
                                break;
                            }
                        }
                        if would_cycle {
                            break;
                        }
                    }
                }

                // Draw connection line in red if it would create a cycle
                let connection_color = if would_cycle {
                    Color32::from_rgb(255, 80, 80)  // Red for invalid
                } else {
                    Color32::from_rgb(200, 200, 100)  // Yellow for valid
                };
                self.draw_bezier(&painter, from_pos, to_pos, connection_color, 2.0);

                // Show instruction (with cycle warning if applicable)
                let instruction = if would_cycle {
                    format!("Cannot connect: would create a cycle{}",
                        hover_block_name.map(|n| format!(" with '{}'", n)).unwrap_or_default())
                } else {
                    "Click on an input port to complete connection, or press ESC to cancel".to_string()
                };
                let text_color = if would_cycle { Color32::RED } else { Color32::YELLOW };
                painter.text(
                    rect.center_top() + Vec2::new(0.0, 30.0),
                    egui::Align2::CENTER_CENTER,
                    instruction,
                    egui::FontId::proportional(14.0),
                    text_color,
                );
            }
        }

        // Draw blocks
        let block_ids: Vec<_> = self.pipeline.blocks.keys().cloned().collect();
        for id in block_ids {
            if let Some(block) = self.pipeline.blocks.get(&id) {
                self.draw_block(&painter, block, rect);
            }
        }

        // Draw selection rectangle or lasso
        match self.selection_mode {
            SelectionMode::Rectangle => {
                if let (Some(start), Some(current)) = (self.selection_start, pointer_pos) {
                    let sel_rect = Rect::from_two_pos(start, current);
                    painter.rect_stroke(
                        sel_rect,
                        0.0,
                        Stroke::new(1.5, Color32::from_rgba_unmultiplied(100, 150, 255, 200)),
                    );
                    painter.rect_filled(
                        sel_rect,
                        0.0,
                        Color32::from_rgba_unmultiplied(100, 150, 255, 30),
                    );
                }
            }
            SelectionMode::Lasso => {
                if self.lasso_points.len() >= 2 {
                    // Draw the lasso path
                    let points = self.lasso_points.clone();
                    painter.add(PathShape::line(
                        points.clone(),
                        Stroke::new(2.0, Color32::from_rgba_unmultiplied(255, 150, 100, 200)),
                    ));

                    // Draw closing line to start point
                    if let (Some(first), Some(last)) = (points.first(), points.last()) {
                        painter.line_segment(
                            [*last, *first],
                            Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 150, 100, 100)),
                        );
                    }
                }
            }
            SelectionMode::None => {}
        }

        // Handle port clicks for connection creation
        if response.clicked() {
            if let Some(click_pos) = pointer_pos {
                // Check if clicking on a port
                let mut port_clicked = false;
                let mut connection_to_make: Option<(BlockId, u32, BlockId, u32)> = None;

                // First check if we're finishing a connection
                if self.connecting_from.is_some() {
                    // Collect port info for both left and top positions (input ports)
                    let port_info: Vec<_> = self.pipeline.blocks.iter()
                        .flat_map(|(id, block)| {
                            let block_rect = self.block_rect(block, rect);
                            let num_inputs = block.block_type.num_inputs();
                            (0..num_inputs)
                                .flat_map(|port| {
                                    // Left side position
                                    let pos_left = self.block_input_pos(block, port, rect);
                                    // Top side position
                                    let spacing = block_rect.width() / (num_inputs + 1) as f32;
                                    let pos_top = Pos2::new(block_rect.left() + spacing * (port + 1) as f32, block_rect.top());
                                    vec![(*id, port, pos_left), (*id, port, pos_top)]
                                })
                                .collect::<Vec<_>>()
                        })
                        .collect();

                    for (id, port, port_pos) in port_info {
                        if (click_pos - port_pos).length() < 10.0 * self.zoom {
                            // Complete the connection
                            if let Some((from_block, from_port)) = self.connecting_from.take() {
                                connection_to_make = Some((from_block, from_port, id, port));
                            }
                            port_clicked = true;
                            break;
                        }
                    }
                } else {
                    // Check if starting a connection from an output port
                    // Collect port info for both right and bottom positions (output ports)
                    let port_info: Vec<_> = self.pipeline.blocks.iter()
                        .flat_map(|(id, block)| {
                            let block_rect = self.block_rect(block, rect);
                            let num_outputs = block.block_type.num_outputs();
                            (0..num_outputs)
                                .flat_map(|port| {
                                    // Right side position
                                    let pos_right = self.block_output_pos(block, port, rect);
                                    // Bottom side position
                                    let spacing = block_rect.width() / (num_outputs + 1) as f32;
                                    let pos_bottom = Pos2::new(block_rect.left() + spacing * (port + 1) as f32, block_rect.bottom());
                                    vec![(*id, port, pos_right), (*id, port, pos_bottom)]
                                })
                                .collect::<Vec<_>>()
                        })
                        .collect();

                    for (id, port, port_pos) in port_info {
                        if (click_pos - port_pos).length() < 10.0 * self.zoom {
                            self.connecting_from = Some((id, port));
                            self.selected_blocks.clear();
                            self.selected_connection = None;
                            port_clicked = true;
                            break;
                        }
                    }
                }

                // Now make the connection if needed (with cycle prevention)
                if let Some((from_block, from_port, to_block, to_port)) = connection_to_make {
                    if self.pipeline.would_create_cycle(from_block, to_block) {
                        // Reject connection and show warning
                        self.status_message = Some((
                            "Cannot create connection: would form a cycle".to_string(),
                            std::time::Instant::now(),
                        ));
                    } else {
                        self.pipeline.connect(from_block, from_port, to_block, to_port);
                    }
                }

                if !port_clicked {
                    // Cancel connection if clicking elsewhere
                    self.connecting_from = None;

                    // Check if clicking on a connection
                    let mut clicked_connection = None;
                    for (idx, conn) in self.pipeline.connections.iter().enumerate() {
                        if let (Some(from_block), Some(to_block)) = (
                            self.pipeline.blocks.get(&conn.from_block),
                            self.pipeline.blocks.get(&conn.to_block),
                        ) {
                            let from_pos = self.block_output_pos(from_block, conn.from_port, rect);
                            let to_pos = self.block_input_pos(to_block, conn.to_port, rect);
                            if self.point_near_bezier(click_pos, from_pos, to_pos, 8.0) {
                                clicked_connection = Some(idx);
                                break;
                            }
                        }
                    }

                    if let Some(idx) = clicked_connection {
                        self.selected_connection = Some(idx);
                        self.selected_blocks.clear();
                    } else {
                        // Check if clicking on a block
                        let mut clicked_block = None;
                        for (id, block) in &self.pipeline.blocks {
                            let block_rect = self.block_rect(block, rect);
                            if block_rect.contains(click_pos) {
                                clicked_block = Some(*id);
                                break;
                            }
                        }

                        if let Some(id) = clicked_block {
                            // Check for shift modifier for multi-select
                            let shift_held = ui.input(|i| i.modifiers.shift);
                            if shift_held {
                                self.toggle_block_selection(id);
                            } else {
                                self.select_single_block(id);
                            }
                        } else {
                            // Clicked on empty canvas - just clear selection (don't start selection mode)
                            // Selection mode only starts on drag, not click
                            let shift_held = ui.input(|i| i.modifiers.shift);
                            if !shift_held {
                                self.selected_blocks.clear();
                            }
                            self.selected_connection = None;
                        }
                    }
                }
            }
        }

        // Handle drag start - select block under cursor or start selection mode
        if response.drag_started() {
            if let Some(start_pos) = pointer_pos {
                // Check if starting drag on a block
                let mut block_under_cursor = None;
                for (id, block) in &self.pipeline.blocks {
                    let block_rect = self.block_rect(block, rect);
                    if block_rect.contains(start_pos) {
                        block_under_cursor = Some(*id);
                        break;
                    }
                }

                if let Some(id) = block_under_cursor {
                    // Starting drag on a block - select it if not already selected
                    if !self.selected_blocks.contains(&id) {
                        let shift_held = ui.input(|i| i.modifiers.shift);
                        if shift_held {
                            self.selected_blocks.insert(id);
                        } else {
                            self.select_single_block(id);
                        }
                    }
                    // We're dragging blocks, not doing selection
                    self.selection_mode = SelectionMode::None;
                    self.selection_start = None;
                    self.lasso_points.clear();

                    // Store initial drag state for accurate tracking
                    self.drag_start_pointer = Some(start_pos);

                    // Calculate all blocks that will be moved
                    let mut blocks_to_move: HashSet<BlockId> = self.selected_blocks.clone();
                    if self.cascade_drag {
                        for &sel_id in &self.selected_blocks {
                            blocks_to_move.extend(self.get_downstream_blocks(sel_id));
                        }
                    }

                    // Store initial positions
                    self.drag_initial_positions.clear();
                    for &block_id in &blocks_to_move {
                        if let Some(block) = self.pipeline.blocks.get(&block_id) {
                            self.drag_initial_positions.insert(block_id, block.position);
                        }
                    }
                } else {
                    // Starting drag on empty canvas - start selection mode
                    let shift_held = ui.input(|i| i.modifiers.shift);
                    let alt_held = ui.input(|i| i.modifiers.alt);

                    if alt_held {
                        // Alt+drag starts lasso selection
                        self.selection_mode = SelectionMode::Lasso;
                        self.selection_start = Some(start_pos);
                        self.lasso_points = vec![start_pos];
                    } else {
                        // Regular drag starts rectangle selection
                        self.selection_mode = SelectionMode::Rectangle;
                        self.selection_start = Some(start_pos);
                    }

                    if !shift_held {
                        self.selected_blocks.clear();
                    }
                    self.selected_connection = None;
                }
            }
        }

        // Handle dragging (block movement or selection)
        if response.dragged() && response.drag_delta().length() > 0.0 {
            if self.connecting_from.is_none() {
                match self.selection_mode {
                    SelectionMode::Lasso => {
                        // Add point to lasso
                        if let Some(pos) = pointer_pos {
                            self.lasso_points.push(pos);
                        }
                    }
                    SelectionMode::Rectangle => {
                        // Rectangle is defined by start and current pointer - no special handling needed
                    }
                    SelectionMode::None => {
                        // Move selected blocks using position-based tracking (not delta accumulation)
                        if !self.drag_initial_positions.is_empty() {
                            if let (Some(start_ptr), Some(current_ptr)) = (self.drag_start_pointer, pointer_pos) {
                                // Calculate total offset from drag start
                                let total_offset = (current_ptr - start_ptr) / self.zoom;

                                // Move all blocks relative to their initial positions
                                for (&block_id, &initial_pos) in &self.drag_initial_positions {
                                    if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                                        let mut new_pos = initial_pos + total_offset;
                                        if self.snap_to_grid {
                                            new_pos.x = (new_pos.x / self.grid_size).round() * self.grid_size;
                                            new_pos.y = (new_pos.y / self.grid_size).round() * self.grid_size;
                                        }
                                        block.position = new_pos;
                                    }
                                }
                            }
                        } else if self.selected_blocks.is_empty() {
                            // No blocks being dragged - pan the canvas
                            self.canvas_offset += response.drag_delta();
                        }
                    }
                }
            }
        }

        // Handle drag release (finalize selection)
        if response.drag_stopped() {
            match self.selection_mode {
                SelectionMode::Rectangle => {
                    if let Some(start) = self.selection_start {
                        if let Some(end) = pointer_pos {
                            let sel_rect = Rect::from_two_pos(start, end);
                            let blocks = self.blocks_in_rect(sel_rect, rect);
                            self.selected_blocks.extend(blocks);
                        }
                    }
                    self.selection_mode = SelectionMode::None;
                    self.selection_start = None;
                }
                SelectionMode::Lasso => {
                    if self.lasso_points.len() >= 3 {
                        let blocks = self.blocks_in_lasso(&self.lasso_points, rect);
                        self.selected_blocks.extend(blocks);
                    }
                    self.selection_mode = SelectionMode::None;
                    self.selection_start = None;
                    self.lasso_points.clear();
                }
                SelectionMode::None => {
                    // Clear block drag state
                    self.drag_start_pointer = None;
                    self.drag_initial_positions.clear();
                }
            }
        }

        // Keyboard shortcuts
        ui.input(|i| {
            // ESC to cancel connection or selection
            if i.key_pressed(egui::Key::Escape) {
                self.connecting_from = None;
                self.clear_selection();
            }

            // Ctrl+A to select all
            if i.modifiers.command && i.key_pressed(egui::Key::A) {
                self.selected_blocks = self.pipeline.blocks.keys().copied().collect();
            }

            // Delete selected blocks or connection
            if i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace) {
                if !self.selected_blocks.is_empty() {
                    let to_delete: Vec<_> = self.selected_blocks.iter().copied().collect();
                    for id in to_delete {
                        self.pipeline.remove_block(id);
                    }
                    self.selected_blocks.clear();
                } else if let Some(idx) = self.selected_connection.take() {
                    if idx < self.pipeline.connections.len() {
                        self.pipeline.connections.remove(idx);
                    }
                }
            }
        });

        // Zoom with scroll - only when hovering over canvas (not floating windows)
        if response.hovered() {
            ui.input(|i| {
                let scroll = i.raw_scroll_delta.y;
                if scroll != 0.0 {
                    self.zoom = (self.zoom + scroll * 0.001).clamp(0.5, 2.0);
                }
            });
        }

        // Context menu for right-click
        response.context_menu(|ui| {
            if !self.selected_blocks.is_empty() {
                let count = self.selected_blocks.len();
                if count == 1 {
                    let id = *self.selected_blocks.iter().next().unwrap();
                    if ui.button("Duplicate Block").clicked() {
                        if let Some(block) = self.pipeline.blocks.get(&id) {
                            let new_pos = block.position + Vec2::new(20.0, 20.0);
                            let new_type = block.block_type.clone();
                            self.pipeline.add_block(new_type, new_pos);
                        }
                        ui.close_menu();
                    }
                    if ui.button("Delete Block").clicked() {
                        self.pipeline.remove_block(id);
                        self.selected_blocks.clear();
                        ui.close_menu();
                    }
                    if ui.button("Disconnect All").clicked() {
                        self.pipeline.connections.retain(|c| c.from_block != id && c.to_block != id);
                        ui.close_menu();
                    }
                } else {
                    if ui.button(format!("Delete {} Blocks", count)).clicked() {
                        let to_delete: Vec<_> = self.selected_blocks.iter().copied().collect();
                        for id in to_delete {
                            self.pipeline.remove_block(id);
                        }
                        self.selected_blocks.clear();
                        ui.close_menu();
                    }
                    if ui.button("Disconnect All").clicked() {
                        let ids: HashSet<_> = self.selected_blocks.clone();
                        self.pipeline.connections.retain(|c| !ids.contains(&c.from_block) && !ids.contains(&c.to_block));
                        ui.close_menu();
                    }
                }
            } else if let Some(idx) = self.selected_connection {
                if ui.button("Delete Connection").clicked() {
                    if idx < self.pipeline.connections.len() {
                        self.pipeline.connections.remove(idx);
                    }
                    self.selected_connection = None;
                    ui.close_menu();
                }
            } else {
                ui.label("Right-click on a block or connection for options");
            }
        });

        // Connection mode indicator
        let mode_text = if self.connecting_from.is_some() {
            "MODE: Connecting...".to_string()
        } else if self.selection_mode != SelectionMode::None {
            format!("MODE: {:?} Selection", self.selection_mode)
        } else if !self.selected_blocks.is_empty() {
            if self.selected_blocks.len() == 1 {
                "Selected: 1 Block".to_string()
            } else {
                format!("Selected: {} Blocks", self.selected_blocks.len())
            }
        } else if self.selected_connection.is_some() {
            "Selected: Connection".to_string()
        } else {
            String::new()
        };

        if !mode_text.is_empty() {
            painter.text(
                rect.right_top() + Vec2::new(-10.0, 10.0),
                egui::Align2::RIGHT_TOP,
                mode_text,
                egui::FontId::proportional(12.0),
                Color32::YELLOW,
            );
        }

        // Instructions
        painter.text(
            rect.left_bottom() + Vec2::new(10.0, -10.0),
            egui::Align2::LEFT_BOTTOM,
            "Click output ports to connect | Drag blocks to move | Delete/Backspace to remove | Scroll to zoom | Right-click for menu | Drag empty area: rect select | Alt+drag: lasso | Shift+click: multi-select | Ctrl+A: select all",
            egui::FontId::proportional(10.0),
            Color32::GRAY,
        );

        // Stats
        painter.text(
            rect.right_bottom() + Vec2::new(-10.0, -10.0),
            egui::Align2::RIGHT_BOTTOM,
            format!("Blocks: {} | Connections: {} | Zoom: {:.0}%",
                self.pipeline.blocks.len(),
                self.pipeline.connections.len(),
                self.zoom * 100.0),
            egui::FontId::proportional(11.0),
            Color32::GRAY,
        );

        // Pipeline title at top center (drawn last to appear on top)
        if !self.pipeline.name.is_empty() {
            painter.text(
                Pos2::new(rect.center().x, rect.top() + 20.0),
                egui::Align2::CENTER_CENTER,
                &self.pipeline.name,
                egui::FontId::proportional(18.0),
                Color32::from_rgb(200, 200, 220),
            );
        }
    }

    fn draw_bezier(&self, painter: &egui::Painter, from_pos: Pos2, to_pos: Pos2, color: Color32, width: f32) {
        let ctrl1 = Pos2::new(from_pos.x + 50.0 * self.zoom, from_pos.y);
        let ctrl2 = Pos2::new(to_pos.x - 50.0 * self.zoom, to_pos.y);

        let points: Vec<Pos2> = (0..=20).map(|i| {
            let t = i as f32 / 20.0;
            let t2 = t * t;
            let t3 = t2 * t;
            let mt = 1.0 - t;
            let mt2 = mt * mt;
            let mt3 = mt2 * mt;

            Pos2::new(
                mt3 * from_pos.x + 3.0 * mt2 * t * ctrl1.x + 3.0 * mt * t2 * ctrl2.x + t3 * to_pos.x,
                mt3 * from_pos.y + 3.0 * mt2 * t * ctrl1.y + 3.0 * mt * t2 * ctrl2.y + t3 * to_pos.y,
            )
        }).collect();

        painter.add(PathShape::line(points, Stroke::new(width, color)));
    }

    fn point_near_bezier(&self, point: Pos2, from_pos: Pos2, to_pos: Pos2, threshold: f32) -> bool {
        let ctrl1 = Pos2::new(from_pos.x + 50.0 * self.zoom, from_pos.y);
        let ctrl2 = Pos2::new(to_pos.x - 50.0 * self.zoom, to_pos.y);

        for i in 0..=20 {
            let t = i as f32 / 20.0;
            let t2 = t * t;
            let t3 = t2 * t;
            let mt = 1.0 - t;
            let mt2 = mt * mt;
            let mt3 = mt2 * mt;

            let curve_point = Pos2::new(
                mt3 * from_pos.x + 3.0 * mt2 * t * ctrl1.x + 3.0 * mt * t2 * ctrl2.x + t3 * to_pos.x,
                mt3 * from_pos.y + 3.0 * mt2 * t * ctrl1.y + 3.0 * mt * t2 * ctrl2.y + t3 * to_pos.y,
            );

            if (point - curve_point).length() < threshold {
                return true;
            }
        }
        false
    }

    fn draw_block(&self, painter: &egui::Painter, block: &PipelineBlock, canvas_rect: Rect) {
        let block_rect = self.block_rect(block, canvas_rect);
        let category = block.block_type.category();
        let is_selected = self.selected_blocks.contains(&block.id);
        let is_disabled = !block.enabled;

        // Block shadow
        let shadow_rect = block_rect.translate(Vec2::new(3.0, 3.0));
        painter.rect_filled(shadow_rect, 5.0, Color32::from_rgba_unmultiplied(0, 0, 0, 60));

        // Block background
        let bg_color = if is_disabled {
            Color32::from_rgb(60, 60, 60)
        } else if is_selected {
            category.color().gamma_multiply(0.6)
        } else {
            category.color().gamma_multiply(0.4)
        };
        painter.rect_filled(block_rect, 5.0, bg_color);

        // Border
        let border_color = if is_selected {
            Color32::WHITE
        } else if is_disabled {
            Color32::GRAY
        } else {
            category.color()
        };
        let border_width = if is_selected { 3.0 } else { 2.0 };
        painter.rect_stroke(block_rect, 5.0, Stroke::new(border_width, border_color));

        // Category indicator bar at top
        let indicator_rect = Rect::from_min_size(
            block_rect.left_top(),
            Vec2::new(block_rect.width(), 4.0 * self.zoom)
        );
        painter.rect_filled(indicator_rect, egui::Rounding::same(5.0), category.color());

        // Block name
        let text_color = if is_disabled { Color32::GRAY } else { Color32::WHITE };
        painter.text(
            block_rect.center(),
            egui::Align2::CENTER_CENTER,
            &block.name,
            egui::FontId::proportional(11.0 * self.zoom),
            text_color,
        );

        // Block ID (small, in corner)
        painter.text(
            block_rect.right_bottom() + Vec2::new(-5.0, -3.0),
            egui::Align2::RIGHT_BOTTOM,
            format!("#{}", block.id),
            egui::FontId::proportional(8.0 * self.zoom),
            Color32::from_rgb(150, 150, 150),
        );

        // Input ports - show only connected ports (on the correct side), OR all when in connection mode
        let port_radius = 6.0 * self.zoom;
        let is_connecting = self.connecting_from.is_some();
        let num_inputs = block.block_type.num_inputs();
        for i in 0..num_inputs {
            let is_connected_horiz = self.is_input_connected_horizontal(block.id, i, canvas_rect);
            let is_connected_vert = self.is_input_connected_vertical(block.id, i, canvas_rect);

            let port_color = if is_connecting {
                Color32::from_rgb(100, 200, 255) // Highlight when connecting
            } else {
                Color32::from_rgb(80, 120, 200)
            };

            // Left side port (horizontal connections)
            // Show if: connected horizontally, OR in connection mode (as potential target)
            if is_connected_horiz || is_connecting {
                let port_pos_left = self.block_input_pos(block, i, canvas_rect);
                painter.circle_stroke(port_pos_left, port_radius + 2.0, Stroke::new(1.0, port_color));
                painter.circle_filled(port_pos_left, port_radius, port_color);
                painter.circle_filled(port_pos_left, port_radius * 0.4, Color32::WHITE);
            }

            // Top side port (vertical connections) - same styling as left port
            // Show if: connected vertically, OR in connection mode (as potential target)
            if is_connected_vert || is_connecting {
                let spacing = block_rect.width() / (num_inputs + 1) as f32;
                let port_pos_top = Pos2::new(block_rect.left() + spacing * (i + 1) as f32, block_rect.top());
                painter.circle_stroke(port_pos_top, port_radius + 2.0, Stroke::new(1.0, port_color));
                painter.circle_filled(port_pos_top, port_radius, port_color);
                painter.circle_filled(port_pos_top, port_radius * 0.4, Color32::WHITE);
            }
        }

        // Output ports - show only connected ports on the correct side
        // If no connections exist, show right port (default for starting new connections)
        let num_outputs = block.block_type.num_outputs();
        for i in 0..num_outputs {
            let is_connected_horiz = self.is_output_connected_horizontal(block.id, i, canvas_rect);
            let is_connected_vert = self.is_output_connected_vertical(block.id, i, canvas_rect);
            let has_any_connection = is_connected_horiz || is_connected_vert;
            let is_connection_source = self.connecting_from == Some((block.id, i));

            let port_color = if is_connection_source {
                Color32::from_rgb(255, 200, 100) // Highlight when this is source
            } else {
                Color32::from_rgb(200, 80, 80)
            };

            // Right side port (horizontal connections)
            // Show if: connected horizontally, OR is source, OR (not connecting AND no connections exist)
            let show_right = is_connected_horiz || is_connection_source || (!is_connecting && !has_any_connection);
            if show_right {
                let port_pos_right = self.block_output_pos(block, i, canvas_rect);
                painter.circle_stroke(port_pos_right, port_radius + 2.0, Stroke::new(1.0, port_color));
                painter.circle_filled(port_pos_right, port_radius, port_color);
                painter.circle_filled(port_pos_right, port_radius * 0.4, Color32::WHITE);
            }

            // Bottom side port (vertical connections) - same styling as right port
            // Show if: connected vertically, OR is source
            let show_bottom = is_connected_vert || is_connection_source;
            if show_bottom {
                let spacing = block_rect.width() / (num_outputs + 1) as f32;
                let port_pos_bottom = Pos2::new(block_rect.left() + spacing * (i + 1) as f32, block_rect.bottom());
                painter.circle_stroke(port_pos_bottom, port_radius + 2.0, Stroke::new(1.0, port_color));
                painter.circle_filled(port_pos_bottom, port_radius, port_color);
                painter.circle_filled(port_pos_bottom, port_radius * 0.4, Color32::WHITE);
            }
        }

        // Disabled overlay
        if is_disabled {
            painter.line_segment(
                [block_rect.left_top(), block_rect.right_bottom()],
                Stroke::new(2.0, Color32::RED),
            );
        }
    }

    fn block_rect(&self, block: &PipelineBlock, canvas_rect: Rect) -> Rect {
        let size = Vec2::new(120.0, 50.0) * self.zoom;
        let pos = canvas_rect.left_top() + self.canvas_offset + block.position.to_vec2() * self.zoom;
        Rect::from_min_size(pos, size)
    }

    fn block_input_pos(&self, block: &PipelineBlock, port: u32, canvas_rect: Rect) -> Pos2 {
        let block_rect = self.block_rect(block, canvas_rect);
        let num_inputs = block.block_type.num_inputs();
        let spacing = block_rect.height() / (num_inputs + 1) as f32;
        Pos2::new(block_rect.left(), block_rect.top() + spacing * (port + 1) as f32)
    }

    fn block_output_pos(&self, block: &PipelineBlock, port: u32, canvas_rect: Rect) -> Pos2 {
        let block_rect = self.block_rect(block, canvas_rect);
        let num_outputs = block.block_type.num_outputs();
        let spacing = block_rect.height() / (num_outputs + 1) as f32;
        Pos2::new(block_rect.right(), block_rect.top() + spacing * (port + 1) as f32)
    }

    /// Determine the best connection orientation based on relative block positions
    /// Returns true if vertical ports (bottom→top) should be used, false for horizontal (right→left)
    fn is_vertical_connection(&self, from_block: &PipelineBlock, to_block: &PipelineBlock, canvas_rect: Rect) -> bool {
        let from_rect = self.block_rect(from_block, canvas_rect);
        let to_rect = self.block_rect(to_block, canvas_rect);

        let from_center = from_rect.center();
        let to_center = to_rect.center();

        let dx = to_center.x - from_center.x;  // positive = target is to the right
        let dy = to_center.y - from_center.y;  // positive = target is below

        // Heuristic decision:
        // 1. If target is significantly below (large positive dy), use vertical (bottom→top)
        // 2. If target is to the left (negative dx) AND below, prefer vertical to avoid crossing
        // 3. If target is to the right and not much vertical difference, use horizontal
        // 4. Consider block overlap to decide

        // Target is below
        if dy > 0.0 {
            // If going backward (left) while going down, always use vertical
            if dx < 0.0 {
                return true;
            }
            // If vertical displacement is significant, use vertical
            if dy > dx.abs() * 0.5 {
                return true;
            }
        }

        // Target is above (unusual but possible)
        if dy < 0.0 {
            // Going backward and up - use horizontal (right side) to avoid confusion
            return false;
        }

        // Default: use horizontal for rightward connections, vertical for significant downward
        dy.abs() > dx.abs() && dy > 0.0
    }

    /// Get output port position adapted for the target block's location
    fn block_output_pos_for_connection(&self, block: &PipelineBlock, port: u32, target_block: &PipelineBlock, canvas_rect: Rect) -> Pos2 {
        let block_rect = self.block_rect(block, canvas_rect);
        let num_outputs = block.block_type.num_outputs();

        if self.is_vertical_connection(block, target_block, canvas_rect) {
            // Output on bottom for vertical connections
            let spacing = block_rect.width() / (num_outputs + 1) as f32;
            Pos2::new(block_rect.left() + spacing * (port + 1) as f32, block_rect.bottom())
        } else {
            // Output on right for horizontal connections (default)
            let spacing = block_rect.height() / (num_outputs + 1) as f32;
            Pos2::new(block_rect.right(), block_rect.top() + spacing * (port + 1) as f32)
        }
    }

    /// Get input port position adapted for the source block's location
    fn block_input_pos_for_connection(&self, block: &PipelineBlock, port: u32, source_block: &PipelineBlock, canvas_rect: Rect) -> Pos2 {
        let block_rect = self.block_rect(block, canvas_rect);
        let num_inputs = block.block_type.num_inputs();

        if self.is_vertical_connection(source_block, block, canvas_rect) {
            // Input on top for vertical connections
            let spacing = block_rect.width() / (num_inputs + 1) as f32;
            Pos2::new(block_rect.left() + spacing * (port + 1) as f32, block_rect.top())
        } else {
            // Input on left for horizontal connections (default)
            let spacing = block_rect.height() / (num_inputs + 1) as f32;
            Pos2::new(block_rect.left(), block_rect.top() + spacing * (port + 1) as f32)
        }
    }

    /// Draw a connection line with the current style
    fn draw_connection(&self, painter: &egui::Painter, from_pos: Pos2, to_pos: Pos2, is_vertical: bool, color: Color32, width: f32) {
        let port_radius = 6.0 * self.zoom;
        let arrow_size = 8.0 * self.zoom;

        // Calculate the effective endpoint (shortened if arrowheads are shown)
        // Use the arrival direction based on connection orientation, not the straight-line direction
        let effective_to = if self.show_arrowheads {
            // For bezier/curved connections, the line arrives perpendicular to the port
            // Vertical: arrives from above (pointing down into top port)
            // Horizontal: arrives from left (pointing right into left port)
            let arrival_dir = if is_vertical {
                Vec2::new(0.0, 1.0) // Coming from above, pointing down
            } else {
                Vec2::new(1.0, 0.0) // Coming from left, pointing right
            };
            to_pos - arrival_dir * (port_radius + arrow_size)
        } else {
            to_pos
        };

        // Get the point before the destination for arrowhead direction
        let arrow_from = match self.connection_style {
            ConnectionStyle::Bezier => {
                self.draw_bezier_connection(painter, from_pos, effective_to, is_vertical, color, width);
                // For bezier, approximate with control point direction
                let offset = 50.0 * self.zoom;
                if is_vertical {
                    Pos2::new(to_pos.x, to_pos.y - offset)
                } else {
                    Pos2::new(to_pos.x - offset, to_pos.y)
                }
            }
            ConnectionStyle::Straight => {
                painter.line_segment([from_pos, effective_to], Stroke::new(width, color));
                from_pos
            }
            ConnectionStyle::Orthogonal => {
                self.draw_orthogonal_connection(painter, from_pos, effective_to, is_vertical, color, width);
                // Last segment is horizontal or vertical
                let min_offset = 20.0 * self.zoom;
                if is_vertical {
                    let mid_y = if to_pos.y > from_pos.y {
                        from_pos.y + (to_pos.y - from_pos.y) / 2.0
                    } else {
                        from_pos.y + min_offset
                    };
                    Pos2::new(to_pos.x, mid_y)
                } else {
                    let mid_x = if to_pos.x > from_pos.x {
                        from_pos.x + (to_pos.x - from_pos.x) / 2.0
                    } else {
                        from_pos.x + min_offset
                    };
                    Pos2::new(mid_x, to_pos.y)
                }
            }
            ConnectionStyle::Angled => {
                self.draw_angled_connection(painter, from_pos, effective_to, is_vertical, color, width);
                // Estimate the last segment direction
                let dx = to_pos.x - from_pos.x;
                let dy = to_pos.y - from_pos.y;
                if is_vertical {
                    let angle_dist = dx.abs().min(dy.abs() / 2.0);
                    if dy > 0.0 {
                        Pos2::new(from_pos.x, from_pos.y + (dy - angle_dist))
                    } else {
                        from_pos
                    }
                } else {
                    let angle_dist = dy.abs().min(dx.abs() / 2.0);
                    if dx > 0.0 {
                        Pos2::new(from_pos.x + (dx - angle_dist), from_pos.y)
                    } else {
                        from_pos
                    }
                }
            }
        };

        // Draw arrowhead if enabled
        if self.show_arrowheads {
            self.draw_arrowhead(painter, to_pos, arrow_from, color);
        }
    }

    /// Draw a smooth bezier curve connection
    fn draw_bezier_connection(&self, painter: &egui::Painter, from_pos: Pos2, to_pos: Pos2, is_vertical: bool, color: Color32, width: f32) {
        let offset = 50.0 * self.zoom;

        let (ctrl1, ctrl2) = if is_vertical {
            // Vertical connection: control points extend downward then upward
            (
                Pos2::new(from_pos.x, from_pos.y + offset),
                Pos2::new(to_pos.x, to_pos.y - offset),
            )
        } else {
            // Horizontal connection: control points extend right then left
            (
                Pos2::new(from_pos.x + offset, from_pos.y),
                Pos2::new(to_pos.x - offset, to_pos.y),
            )
        };

        let points: Vec<Pos2> = (0..=20).map(|i| {
            let t = i as f32 / 20.0;
            let t2 = t * t;
            let t3 = t2 * t;
            let mt = 1.0 - t;
            let mt2 = mt * mt;
            let mt3 = mt2 * mt;

            Pos2::new(
                mt3 * from_pos.x + 3.0 * mt2 * t * ctrl1.x + 3.0 * mt * t2 * ctrl2.x + t3 * to_pos.x,
                mt3 * from_pos.y + 3.0 * mt2 * t * ctrl1.y + 3.0 * mt * t2 * ctrl2.y + t3 * to_pos.y,
            )
        }).collect();

        painter.add(PathShape::line(points, Stroke::new(width, color)));
    }

    /// Draw a right-angle (Manhattan) connection with two turns
    fn draw_orthogonal_connection(&self, painter: &egui::Painter, from_pos: Pos2, to_pos: Pos2, is_vertical: bool, color: Color32, width: f32) {
        let min_offset = 20.0 * self.zoom;

        let points = if is_vertical {
            // Vertical: go down, then horizontal, then down to target
            let mid_y = if to_pos.y > from_pos.y {
                from_pos.y + (to_pos.y - from_pos.y) / 2.0
            } else {
                from_pos.y + min_offset
            };
            vec![
                from_pos,
                Pos2::new(from_pos.x, mid_y),
                Pos2::new(to_pos.x, mid_y),
                to_pos,
            ]
        } else {
            // Horizontal: go right, then vertical, then right to target
            let mid_x = if to_pos.x > from_pos.x {
                from_pos.x + (to_pos.x - from_pos.x) / 2.0
            } else {
                from_pos.x + min_offset
            };
            vec![
                from_pos,
                Pos2::new(mid_x, from_pos.y),
                Pos2::new(mid_x, to_pos.y),
                to_pos,
            ]
        };

        painter.add(PathShape::line(points, Stroke::new(width, color)));
    }

    /// Draw a 45-degree angled connection
    fn draw_angled_connection(&self, painter: &egui::Painter, from_pos: Pos2, to_pos: Pos2, is_vertical: bool, color: Color32, width: f32) {
        let dx = to_pos.x - from_pos.x;
        let dy = to_pos.y - from_pos.y;

        let points = if is_vertical {
            // Vertical: go straight down first, then 45° to target
            let angle_dist = dx.abs().min(dy.abs() / 2.0);
            if dy > 0.0 {
                let mid_y = from_pos.y + (dy - angle_dist);
                vec![
                    from_pos,
                    Pos2::new(from_pos.x, mid_y),
                    to_pos,
                ]
            } else {
                vec![from_pos, to_pos]
            }
        } else {
            // Horizontal: go straight right first, then 45° to target
            let angle_dist = dy.abs().min(dx.abs() / 2.0);
            if dx > 0.0 {
                let mid_x = from_pos.x + (dx - angle_dist);
                vec![
                    from_pos,
                    Pos2::new(mid_x, from_pos.y),
                    to_pos,
                ]
            } else {
                vec![from_pos, to_pos]
            }
        };

        painter.add(PathShape::line(points, Stroke::new(width, color)));
    }

    /// Draw an arrowhead at the destination point (stops at port circle edge)
    fn draw_arrowhead(&self, painter: &egui::Painter, tip: Pos2, from_pos: Pos2, color: Color32) {
        // Calculate direction from last segment to tip
        let dir = (tip - from_pos).normalized();
        let arrow_size = 8.0 * self.zoom;
        let port_radius = 6.0 * self.zoom;

        // Offset the tip to end at the port circle perimeter, not the center
        let adjusted_tip = tip - dir * port_radius;

        // Calculate perpendicular vectors for the arrowhead wings
        let perp = Vec2::new(-dir.y, dir.x);

        // Arrow base point (behind the adjusted tip)
        let base = adjusted_tip - dir * arrow_size;

        // Wing points
        let wing1 = base + perp * (arrow_size * 0.5);
        let wing2 = base - perp * (arrow_size * 0.5);

        // Draw filled triangle
        let points = vec![adjusted_tip, wing1, wing2];
        painter.add(egui::Shape::convex_polygon(points, color, Stroke::NONE));
    }

    fn render_properties_content(&mut self, ui: &mut Ui) {
        let selection_count = self.selected_blocks.len();

        if selection_count == 1 {
            // Single block selected - show full properties
            let id = *self.selected_blocks.iter().next().unwrap();
            if let Some(block) = self.pipeline.blocks.get_mut(&id) {
                ui.horizontal(|ui| {
                    ui.label("Name:");
                    ui.text_edit_singleline(&mut block.name);
                });

                ui.checkbox(&mut block.enabled, "Enabled");

                ui.add_space(10.0);
                ui.label(RichText::new("Parameters").strong());
                ui.separator();

                // Render block-specific parameters
                self.render_block_params(ui, id);

                // Render block metadata (documentation, formulas, code links)
                let block_type_name = self.get_block_type_name(id);
                self.render_block_metadata(ui, &block_type_name);

                ui.add_space(10.0);
                if ui.button("Delete Block").clicked() {
                    self.pipeline.remove_block(id);
                    self.selected_blocks.clear();
                }
            }
        } else if selection_count > 1 {
            // Multiple blocks selected
            ui.label(RichText::new(format!("{} Blocks Selected", selection_count)).strong());
            ui.separator();

            // List selected blocks
            for &id in &self.selected_blocks.clone() {
                if let Some(block) = self.pipeline.blocks.get(&id) {
                    ui.horizontal(|ui| {
                        ui.label(format!("#{}: {}", id, block.name));
                    });
                }
            }

            ui.add_space(10.0);
            if ui.button(format!("Delete {} Blocks", selection_count)).clicked() {
                let to_delete: Vec<_> = self.selected_blocks.iter().copied().collect();
                for id in to_delete {
                    self.pipeline.remove_block(id);
                }
                self.selected_blocks.clear();
            }
        } else {
            // Pipeline properties
            ui.label("Pipeline Name:");
            ui.text_edit_singleline(&mut self.pipeline.name);

            ui.add_space(5.0);
            ui.label("Description:");
            ui.text_edit_multiline(&mut self.pipeline.description);

            ui.add_space(10.0);
            ui.label(format!("Blocks: {}", self.pipeline.blocks.len()));
            ui.label(format!("Connections: {}", self.pipeline.connections.len()));

            ui.add_space(10.0);
            ui.label(RichText::new("Instructions").italics());
            ui.label("1. Click blocks in library to add");
            ui.label("2. Select a block to edit");
            ui.label("3. Drag blocks to position");
            ui.label("4. Export to YAML when done");
        }
    }

    fn render_block_params(&mut self, ui: &mut Ui, block_id: BlockId) {
        // We need to get a mutable reference to the block type
        // This is a bit awkward due to borrowing rules
        let block_type = if let Some(block) = self.pipeline.blocks.get(&block_id) {
            block.block_type.clone()
        } else {
            return;
        };

        match block_type {
            BlockType::BitSource { pattern } => {
                let mut new_pattern = pattern;
                ui.horizontal(|ui| {
                    ui.label("Pattern:");
                    egui::ComboBox::from_id_salt("bit_pattern")
                        .selected_text(&new_pattern)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_pattern, "random".to_string(), "Random");
                            ui.selectable_value(&mut new_pattern, "alternating".to_string(), "Alternating");
                            ui.selectable_value(&mut new_pattern, "ones".to_string(), "All Ones");
                            ui.selectable_value(&mut new_pattern, "zeros".to_string(), "All Zeros");
                            ui.selectable_value(&mut new_pattern, "prbs7".to_string(), "PRBS-7");
                            ui.selectable_value(&mut new_pattern, "prbs15".to_string(), "PRBS-15");
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::BitSource { pattern: new_pattern };
                }
            }
            BlockType::SymbolSource { alphabet_size } => {
                let mut new_size = alphabet_size;
                ui.horizontal(|ui| {
                    ui.label("Alphabet:");
                    egui::ComboBox::from_id_salt("sym_alphabet")
                        .selected_text(format!("{}", new_size))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_size, 2, "2 (Binary)");
                            ui.selectable_value(&mut new_size, 4, "4 (Quaternary)");
                            ui.selectable_value(&mut new_size, 8, "8");
                            ui.selectable_value(&mut new_size, 16, "16");
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::SymbolSource { alphabet_size: new_size };
                }
            }
            BlockType::Scrambler { polynomial, seed } => {
                let mut new_poly = polynomial;
                let mut new_seed = seed;
                ui.horizontal(|ui| {
                    ui.label("Polynomial:");
                    ui.add(egui::DragValue::new(&mut new_poly).hexadecimal(2, false, true));
                });
                ui.horizontal(|ui| {
                    ui.label("Seed:");
                    ui.add(egui::DragValue::new(&mut new_seed).hexadecimal(2, false, true));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::Scrambler { polynomial: new_poly, seed: new_seed };
                }
            }
            BlockType::FecEncoder { code_type, rate } => {
                let mut new_type = code_type;
                let mut new_rate = rate;
                ui.label("Code Type:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_type, FecType::Convolutional, "Conv");
                    ui.radio_value(&mut new_type, FecType::Ldpc, "LDPC");
                    ui.radio_value(&mut new_type, FecType::Turbo, "Turbo");
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_type, FecType::ReedSolomon, "RS");
                    ui.radio_value(&mut new_type, FecType::Hamming, "Hamming");
                });
                ui.horizontal(|ui| {
                    ui.label("Rate:");
                    egui::ComboBox::from_id_salt("fec_rate")
                        .selected_text(&new_rate)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_rate, "1/2".to_string(), "1/2");
                            ui.selectable_value(&mut new_rate, "2/3".to_string(), "2/3");
                            ui.selectable_value(&mut new_rate, "3/4".to_string(), "3/4");
                            ui.selectable_value(&mut new_rate, "5/6".to_string(), "5/6");
                            ui.selectable_value(&mut new_rate, "7/8".to_string(), "7/8");
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FecEncoder { code_type: new_type, rate: new_rate };
                }
            }
            BlockType::Interleaver { rows, cols } => {
                let mut new_rows = rows;
                let mut new_cols = cols;
                ui.horizontal(|ui| {
                    ui.label("Rows:");
                    ui.add(egui::DragValue::new(&mut new_rows).range(2..=64));
                });
                ui.horizontal(|ui| {
                    ui.label("Cols:");
                    ui.add(egui::DragValue::new(&mut new_cols).range(2..=64));
                });
                ui.label(format!("Block size: {} bits", new_rows * new_cols));
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::Interleaver { rows: new_rows, cols: new_cols };
                }
            }
            BlockType::CrcGenerator { crc_type } => {
                let mut new_type = crc_type;
                ui.label("CRC Type:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_type, CrcType::Crc8, "CRC-8");
                    ui.radio_value(&mut new_type, CrcType::Crc16Ccitt, "CRC-16 CCITT");
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_type, CrcType::Crc16Ibm, "CRC-16 IBM");
                    ui.radio_value(&mut new_type, CrcType::Crc32, "CRC-32");
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::CrcGenerator { crc_type: new_type };
                }
            }
            BlockType::GrayMapper { bits_per_symbol } => {
                let mut new_bits = bits_per_symbol;
                ui.horizontal(|ui| {
                    ui.label("Bits/Symbol:");
                    ui.add(egui::DragValue::new(&mut new_bits).range(1..=8));
                });
                ui.label(format!("Constellation: {}-ary", 1u32 << new_bits));
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::GrayMapper { bits_per_symbol: new_bits };
                }
            }
            BlockType::ConstellationMapper { constellation } => {
                let mut new_const = constellation;
                ui.label("Constellation:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_const, ConstellationType::Bpsk, "BPSK");
                    ui.radio_value(&mut new_const, ConstellationType::Qpsk, "QPSK");
                    ui.radio_value(&mut new_const, ConstellationType::Psk8, "8-PSK");
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_const, ConstellationType::Qam16, "16-QAM");
                    ui.radio_value(&mut new_const, ConstellationType::Qam64, "64-QAM");
                    ui.radio_value(&mut new_const, ConstellationType::Qam256, "256-QAM");
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::ConstellationMapper { constellation: new_const };
                }
            }
            BlockType::PskModulator { order } => {
                let mut new_order = order;
                ui.horizontal(|ui| {
                    ui.label("Order:");
                    egui::ComboBox::from_id_salt("psk_order")
                        .selected_text(format!("{}-PSK", new_order))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_order, 2, "BPSK");
                            ui.selectable_value(&mut new_order, 4, "QPSK");
                            ui.selectable_value(&mut new_order, 8, "8-PSK");
                            ui.selectable_value(&mut new_order, 16, "16-PSK");
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::PskModulator { order: new_order };
                }
            }
            BlockType::QamModulator { order } => {
                let mut new_order = order;
                ui.horizontal(|ui| {
                    ui.label("Order:");
                    egui::ComboBox::from_id_salt("qam_order")
                        .selected_text(format!("{}-QAM", new_order))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_order, 16, "16-QAM");
                            ui.selectable_value(&mut new_order, 64, "64-QAM");
                            ui.selectable_value(&mut new_order, 256, "256-QAM");
                            ui.selectable_value(&mut new_order, 1024, "1024-QAM");
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::QamModulator { order: new_order };
                }
            }
            BlockType::FskModulator { deviation_hz, order } => {
                let mut new_dev = deviation_hz;
                let mut new_order = order;
                ui.horizontal(|ui| {
                    ui.label("Deviation (Hz):");
                    ui.add(egui::DragValue::new(&mut new_dev).speed(100.0).range(100.0..=50000.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Order:");
                    egui::ComboBox::from_id_salt("fsk_order")
                        .selected_text(format!("{}-FSK", new_order))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_order, 2, "2-FSK");
                            ui.selectable_value(&mut new_order, 4, "4-FSK");
                            ui.selectable_value(&mut new_order, 8, "8-FSK");
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FskModulator { deviation_hz: new_dev, order: new_order };
                }
            }
            BlockType::OfdmModulator { fft_size, cp_len, data_carriers } => {
                let mut new_fft = fft_size;
                let mut new_cp = cp_len;
                let mut new_data = data_carriers;
                ui.horizontal(|ui| {
                    ui.label("FFT Size:");
                    egui::ComboBox::from_id_salt("ofdm_fft")
                        .selected_text(format!("{}", new_fft))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_fft, 64, "64");
                            ui.selectable_value(&mut new_fft, 128, "128");
                            ui.selectable_value(&mut new_fft, 256, "256");
                            ui.selectable_value(&mut new_fft, 512, "512");
                            ui.selectable_value(&mut new_fft, 1024, "1024");
                            ui.selectable_value(&mut new_fft, 2048, "2048");
                        });
                });
                ui.horizontal(|ui| {
                    ui.label("CP Length:");
                    ui.add(egui::DragValue::new(&mut new_cp).range(0..=new_fft/2));
                });
                ui.horizontal(|ui| {
                    ui.label("Data Carriers:");
                    ui.add(egui::DragValue::new(&mut new_data).range(1..=new_fft-1));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::OfdmModulator { fft_size: new_fft, cp_len: new_cp, data_carriers: new_data };
                }
            }
            BlockType::DsssSpread { chips_per_symbol, code_type } => {
                let mut new_chips = chips_per_symbol;
                let mut new_code = code_type;
                ui.horizontal(|ui| {
                    ui.label("Chips/Symbol:");
                    egui::ComboBox::from_id_salt("dsss_chips")
                        .selected_text(format!("{}", new_chips))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_chips, 7, "7");
                            ui.selectable_value(&mut new_chips, 15, "15");
                            ui.selectable_value(&mut new_chips, 31, "31");
                            ui.selectable_value(&mut new_chips, 63, "63");
                            ui.selectable_value(&mut new_chips, 127, "127");
                            ui.selectable_value(&mut new_chips, 255, "255");
                        });
                });
                ui.horizontal(|ui| {
                    ui.label("Code Type:");
                    egui::ComboBox::from_id_salt("dsss_code")
                        .selected_text(&new_code)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_code, "Gold".to_string(), "Gold");
                            ui.selectable_value(&mut new_code, "Kasami".to_string(), "Kasami");
                            ui.selectable_value(&mut new_code, "Walsh".to_string(), "Walsh");
                            ui.selectable_value(&mut new_code, "PN".to_string(), "PN");
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::DsssSpread { chips_per_symbol: new_chips, code_type: new_code };
                }
            }
            BlockType::FhssHop { num_channels, hop_rate } => {
                let mut new_channels = num_channels;
                let mut new_rate = hop_rate;
                ui.horizontal(|ui| {
                    ui.label("Channels:");
                    ui.add(egui::DragValue::new(&mut new_channels).range(2..=256));
                });
                ui.horizontal(|ui| {
                    ui.label("Hop Rate (Hz):");
                    ui.add(egui::DragValue::new(&mut new_rate).speed(10.0).range(1.0..=10000.0));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FhssHop { num_channels: new_channels, hop_rate: new_rate };
                }
            }
            BlockType::CssModulator { sf, bw_hz } => {
                let mut new_sf = sf;
                let mut new_bw = bw_hz;
                ui.horizontal(|ui| {
                    ui.label("Spreading Factor:");
                    ui.add(egui::DragValue::new(&mut new_sf).range(5..=12));
                });
                ui.horizontal(|ui| {
                    ui.label("Bandwidth:");
                    egui::ComboBox::from_id_salt("css_bw")
                        .selected_text(format!("{} kHz", new_bw / 1000))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_bw, 125000, "125 kHz");
                            ui.selectable_value(&mut new_bw, 250000, "250 kHz");
                            ui.selectable_value(&mut new_bw, 500000, "500 kHz");
                        });
                });
                ui.label(format!("Symbols: {} | Bits/symbol: {}", 1u32 << new_sf, new_sf));
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::CssModulator { sf: new_sf, bw_hz: new_bw };
                }
            }
            // Demodulator property editors
            BlockType::PskDemodulator { order } => {
                let mut new_order = order;
                ui.horizontal(|ui| {
                    ui.label("Order:");
                    egui::ComboBox::from_id_salt("psk_demod_order")
                        .selected_text(format!("{}-PSK", new_order))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_order, 2, "BPSK");
                            ui.selectable_value(&mut new_order, 4, "QPSK");
                            ui.selectable_value(&mut new_order, 8, "8-PSK");
                            ui.selectable_value(&mut new_order, 16, "16-PSK");
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::PskDemodulator { order: new_order };
                }
            }
            BlockType::QamDemodulator { order } => {
                let mut new_order = order;
                ui.horizontal(|ui| {
                    ui.label("Order:");
                    egui::ComboBox::from_id_salt("qam_demod_order")
                        .selected_text(format!("{}-QAM", new_order))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_order, 16, "16-QAM");
                            ui.selectable_value(&mut new_order, 64, "64-QAM");
                            ui.selectable_value(&mut new_order, 256, "256-QAM");
                            ui.selectable_value(&mut new_order, 1024, "1024-QAM");
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::QamDemodulator { order: new_order };
                }
            }
            BlockType::FskDemodulator { order } => {
                let mut new_order = order;
                ui.horizontal(|ui| {
                    ui.label("Order:");
                    egui::ComboBox::from_id_salt("fsk_demod_order")
                        .selected_text(format!("{}-FSK", new_order))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_order, 2, "2-FSK");
                            ui.selectable_value(&mut new_order, 4, "4-FSK");
                            ui.selectable_value(&mut new_order, 8, "8-FSK");
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FskDemodulator { order: new_order };
                }
            }
            BlockType::FirFilter { filter_type, cutoff_hz, num_taps } => {
                let mut new_type = filter_type;
                let mut new_cutoff = cutoff_hz;
                let mut new_taps = num_taps;

                ui.label("Filter Type:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_type, FilterType::Lowpass, "LP");
                    ui.radio_value(&mut new_type, FilterType::Highpass, "HP");
                    ui.radio_value(&mut new_type, FilterType::Bandpass, "BP");
                    ui.radio_value(&mut new_type, FilterType::Bandstop, "BS");
                });

                ui.horizontal(|ui| {
                    ui.label("Cutoff (Hz):");
                    ui.add(egui::DragValue::new(&mut new_cutoff).speed(100.0));
                });

                ui.horizontal(|ui| {
                    ui.label("Taps:");
                    ui.add(egui::DragValue::new(&mut new_taps).range(8..=512));
                });

                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FirFilter {
                        filter_type: new_type,
                        cutoff_hz: new_cutoff,
                        num_taps: new_taps
                    };
                }
            }
            BlockType::IirFilter { design, cutoff_hz, order } => {
                let mut new_design = design;
                let mut new_cutoff = cutoff_hz;
                let mut new_order = order;

                ui.label("Design:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_design, IirDesign::Butterworth, "Butter");
                    ui.radio_value(&mut new_design, IirDesign::Chebyshev1, "Cheby I");
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_design, IirDesign::Chebyshev2, "Cheby II");
                    ui.radio_value(&mut new_design, IirDesign::Bessel, "Bessel");
                });

                ui.horizontal(|ui| {
                    ui.label("Cutoff (Hz):");
                    ui.add(egui::DragValue::new(&mut new_cutoff).speed(100.0));
                });

                ui.horizontal(|ui| {
                    ui.label("Order:");
                    ui.add(egui::DragValue::new(&mut new_order).range(1..=12));
                });

                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::IirFilter {
                        design: new_design,
                        cutoff_hz: new_cutoff,
                        order: new_order
                    };
                }
            }
            BlockType::PulseShaper { shape, rolloff, span } => {
                let mut new_shape = shape;
                let mut new_rolloff = rolloff;
                let mut new_span = span;

                ui.label("Shape:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_shape, PulseShape::RootRaisedCosine, "RRC");
                    ui.radio_value(&mut new_shape, PulseShape::RaisedCosine, "RC");
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_shape, PulseShape::Gaussian, "Gauss");
                    ui.radio_value(&mut new_shape, PulseShape::Rectangular, "Rect");
                });

                ui.horizontal(|ui| {
                    ui.label("Rolloff (α):");
                    ui.add(egui::Slider::new(&mut new_rolloff, 0.0..=1.0));
                });

                ui.horizontal(|ui| {
                    ui.label("Span (symbols):");
                    ui.add(egui::DragValue::new(&mut new_span).range(2..=32));
                });

                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::PulseShaper {
                        shape: new_shape,
                        rolloff: new_rolloff,
                        span: new_span
                    };
                }
            }
            BlockType::Upsampler { factor } => {
                let mut new_factor = factor;
                ui.horizontal(|ui| {
                    ui.label("Factor:");
                    ui.add(egui::DragValue::new(&mut new_factor).range(1..=64));
                });
                ui.label(format!("Output rate: {}× input", new_factor));
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::Upsampler { factor: new_factor };
                }
            }
            BlockType::Downsampler { factor } => {
                let mut new_factor = factor;
                ui.horizontal(|ui| {
                    ui.label("Factor:");
                    ui.add(egui::DragValue::new(&mut new_factor).range(1..=64));
                });
                ui.label(format!("Output rate: 1/{} input", new_factor));
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::Downsampler { factor: new_factor };
                }
            }
            BlockType::RationalResampler { up, down } => {
                let mut new_up = up;
                let mut new_down = down;
                ui.horizontal(|ui| {
                    ui.label("Up:");
                    ui.add(egui::DragValue::new(&mut new_up).range(1..=64));
                });
                ui.horizontal(|ui| {
                    ui.label("Down:");
                    ui.add(egui::DragValue::new(&mut new_down).range(1..=64));
                });
                ui.label(format!("Ratio: {}/{} = {:.4}", new_up, new_down, new_up as f32 / new_down as f32));
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::RationalResampler { up: new_up, down: new_down };
                }
            }
            BlockType::PolyphaseResampler { up, down, taps_per_phase } => {
                let mut new_up = up;
                let mut new_down = down;
                let mut new_taps = taps_per_phase;
                ui.horizontal(|ui| {
                    ui.label("Up:");
                    ui.add(egui::DragValue::new(&mut new_up).range(1..=64));
                });
                ui.horizontal(|ui| {
                    ui.label("Down:");
                    ui.add(egui::DragValue::new(&mut new_down).range(1..=64));
                });
                ui.horizontal(|ui| {
                    ui.label("Taps/Phase:");
                    ui.add(egui::DragValue::new(&mut new_taps).range(4..=64));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::PolyphaseResampler { up: new_up, down: new_down, taps_per_phase: new_taps };
                }
            }
            BlockType::PreambleInsert { pattern, length } => {
                let mut new_pattern = pattern;
                let mut new_length = length;
                ui.horizontal(|ui| {
                    ui.label("Pattern:");
                    egui::ComboBox::from_id_salt("preamble_pattern")
                        .selected_text(&new_pattern)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_pattern, "alternating".to_string(), "Alternating");
                            ui.selectable_value(&mut new_pattern, "barker13".to_string(), "Barker-13");
                            ui.selectable_value(&mut new_pattern, "barker11".to_string(), "Barker-11");
                            ui.selectable_value(&mut new_pattern, "ones".to_string(), "All Ones");
                        });
                });
                ui.horizontal(|ui| {
                    ui.label("Length:");
                    ui.add(egui::DragValue::new(&mut new_length).range(4..=256));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::PreambleInsert { pattern: new_pattern, length: new_length };
                }
            }
            BlockType::SyncWordInsert { word } => {
                let mut new_word = word;
                ui.horizontal(|ui| {
                    ui.label("Sync Word:");
                    ui.text_edit_singleline(&mut new_word);
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::SyncWordInsert { word: new_word };
                }
            }
            BlockType::FrameBuilder { header_bits, payload_bits } => {
                let mut new_header = header_bits;
                let mut new_payload = payload_bits;
                ui.horizontal(|ui| {
                    ui.label("Header bits:");
                    ui.add(egui::DragValue::new(&mut new_header).range(0..=256));
                });
                ui.horizontal(|ui| {
                    ui.label("Payload bits:");
                    ui.add(egui::DragValue::new(&mut new_payload).range(1..=8192));
                });
                ui.label(format!("Frame: {} bits", new_header + new_payload));
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FrameBuilder { header_bits: new_header, payload_bits: new_payload };
                }
            }
            BlockType::TdmaFramer { slots, slot_ms } => {
                let mut new_slots = slots;
                let mut new_slot_ms = slot_ms;
                ui.horizontal(|ui| {
                    ui.label("Slots:");
                    ui.add(egui::DragValue::new(&mut new_slots).range(1..=64));
                });
                ui.horizontal(|ui| {
                    ui.label("Slot (ms):");
                    ui.add(egui::DragValue::new(&mut new_slot_ms).speed(0.1).range(0.1..=100.0));
                });
                ui.label(format!("Frame: {:.1} ms", new_slots as f32 * new_slot_ms));
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::TdmaFramer { slots: new_slots, slot_ms: new_slot_ms };
                }
            }
            BlockType::AwgnChannel { snr_db } => {
                let mut new_snr = snr_db;
                ui.horizontal(|ui| {
                    ui.label("SNR (dB):");
                    ui.add(egui::Slider::new(&mut new_snr, -10.0..=40.0));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::AwgnChannel { snr_db: new_snr };
                }
            }
            BlockType::FadingChannel { model, doppler_hz } => {
                let mut new_model = model;
                let mut new_doppler = doppler_hz;
                ui.label("Fading Model:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_model, FadingModel::Rayleigh, "Rayleigh");
                    ui.radio_value(&mut new_model, FadingModel::Rician, "Rician");
                    ui.radio_value(&mut new_model, FadingModel::Nakagami, "Nakagami");
                });
                ui.horizontal(|ui| {
                    ui.label("Doppler (Hz):");
                    ui.add(egui::DragValue::new(&mut new_doppler).speed(1.0).range(0.1..=1000.0));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FadingChannel { model: new_model, doppler_hz: new_doppler };
                }
            }
            BlockType::FrequencyOffset { offset_hz } => {
                let mut new_offset = offset_hz;
                ui.horizontal(|ui| {
                    ui.label("Offset (Hz):");
                    ui.add(egui::DragValue::new(&mut new_offset).speed(10.0));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FrequencyOffset { offset_hz: new_offset };
                }
            }
            BlockType::IqImbalance { gain_db, phase_deg } => {
                let mut new_gain = gain_db;
                let mut new_phase = phase_deg;
                ui.horizontal(|ui| {
                    ui.label("Gain imbalance (dB):");
                    ui.add(egui::Slider::new(&mut new_gain, -3.0..=3.0));
                });
                ui.horizontal(|ui| {
                    ui.label("Phase imbalance (°):");
                    ui.add(egui::Slider::new(&mut new_phase, -10.0..=10.0));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::IqImbalance { gain_db: new_gain, phase_deg: new_phase };
                }
            }
            BlockType::Agc { mode, target_db } => {
                let mut new_mode = mode;
                let mut new_target = target_db;
                ui.label("AGC Mode:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_mode, AgcMode::Fast, "Fast");
                    ui.radio_value(&mut new_mode, AgcMode::Slow, "Slow");
                    ui.radio_value(&mut new_mode, AgcMode::Adaptive, "Adaptive");
                });
                ui.horizontal(|ui| {
                    ui.label("Target (dB):");
                    ui.add(egui::DragValue::new(&mut new_target).speed(0.5).range(-40.0..=0.0));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::Agc { mode: new_mode, target_db: new_target };
                }
            }
            BlockType::TimingRecovery { algorithm, loop_bw } => {
                let mut new_algo = algorithm;
                let mut new_bw = loop_bw;
                ui.label("Algorithm:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_algo, TimingAlgo::EarlyLate, "Early-Late");
                    ui.radio_value(&mut new_algo, TimingAlgo::Gardner, "Gardner");
                });
                ui.radio_value(&mut new_algo, TimingAlgo::MuellerMuller, "Mueller-Müller");
                ui.horizontal(|ui| {
                    ui.label("Loop BW:");
                    ui.add(egui::Slider::new(&mut new_bw, 0.001..=0.1).logarithmic(true));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::TimingRecovery { algorithm: new_algo, loop_bw: new_bw };
                }
            }
            BlockType::CarrierRecovery { algorithm, loop_bw } => {
                let mut new_algo = algorithm;
                let mut new_bw = loop_bw;
                ui.label("Algorithm:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_algo, CarrierAlgo::CostasLoop, "Costas");
                    ui.radio_value(&mut new_algo, CarrierAlgo::DecisionDirected, "DD");
                });
                ui.radio_value(&mut new_algo, CarrierAlgo::PilotAided, "Pilot-Aided");
                ui.horizontal(|ui| {
                    ui.label("Loop BW:");
                    ui.add(egui::Slider::new(&mut new_bw, 0.001..=0.1).logarithmic(true));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::CarrierRecovery { algorithm: new_algo, loop_bw: new_bw };
                }
            }
            BlockType::Equalizer { eq_type, taps, mu } => {
                let mut new_type = eq_type;
                let mut new_taps = taps;
                let mut new_mu = mu;
                ui.label("Type:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_type, EqualizerType::Lms, "LMS");
                    ui.radio_value(&mut new_type, EqualizerType::Rls, "RLS");
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_type, EqualizerType::Cma, "CMA");
                    ui.radio_value(&mut new_type, EqualizerType::Dfe, "DFE");
                });
                ui.horizontal(|ui| {
                    ui.label("Taps:");
                    ui.add(egui::DragValue::new(&mut new_taps).range(3..=63));
                });
                ui.horizontal(|ui| {
                    ui.label("Step size (μ):");
                    ui.add(egui::Slider::new(&mut new_mu, 0.0001..=0.1).logarithmic(true));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::Equalizer { eq_type: new_type, taps: new_taps, mu: new_mu };
                }
            }
            BlockType::FileOutput { path, format } => {
                let mut new_path = path;
                let mut new_format = format;
                ui.horizontal(|ui| {
                    ui.label("Path:");
                    ui.text_edit_singleline(&mut new_path);
                });
                ui.label("Format:");
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_format, OutputFormat::ComplexFloat32, "cf32");
                    ui.radio_value(&mut new_format, OutputFormat::ComplexFloat64, "cf64");
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut new_format, OutputFormat::ComplexInt16, "ci16");
                    ui.radio_value(&mut new_format, OutputFormat::ComplexInt8, "ci8");
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FileOutput { path: new_path, format: new_format };
                }
            }
            BlockType::Split { num_outputs } => {
                let mut new_outputs = num_outputs;
                ui.horizontal(|ui| {
                    ui.label("Outputs:");
                    ui.add(egui::DragValue::new(&mut new_outputs).range(2..=8));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::Split { num_outputs: new_outputs };
                }
            }
            BlockType::Merge { num_inputs } => {
                let mut new_inputs = num_inputs;
                ui.horizontal(|ui| {
                    ui.label("Inputs:");
                    ui.add(egui::DragValue::new(&mut new_inputs).range(2..=8));
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::Merge { num_inputs: new_inputs };
                }
            }
            BlockType::DifferentialEncoder | BlockType::MatchedFilter |
            BlockType::IqOutput | BlockType::BitOutput | BlockType::IqSplit | BlockType::IqMerge => {
                ui.label(RichText::new("No configurable parameters").italics());
            }
            BlockType::FileSource { path } => {
                let mut new_path = path;
                ui.horizontal(|ui| {
                    ui.label("Path:");
                    ui.text_edit_singleline(&mut new_path);
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FileSource { path: new_path };
                }
            }
        }
    }

    /// Get the block type variant name for metadata lookup
    fn get_block_type_name(&self, block_id: BlockId) -> String {
        if let Some(block) = self.pipeline.blocks.get(&block_id) {
            // Extract the enum variant name
            let debug_str = format!("{:?}", block.block_type);
            // Take everything before the first '{' or space
            debug_str.split(['{', ' ']).next().unwrap_or("Unknown").to_string()
        } else {
            "Unknown".to_string()
        }
    }

    /// Render block documentation and metadata
    fn render_block_metadata(&mut self, ui: &mut Ui, block_type_name: &str) {
        if let Some(meta) = get_block_metadata(block_type_name) {
            ui.add_space(15.0);

            // Collapsible documentation section
            egui::CollapsingHeader::new(RichText::new("📚 Documentation").strong())
                .default_open(false)
                .show(ui, |ui| {
                    // Description
                    ui.label(meta.description);

                    // Input/Output types
                    ui.add_space(5.0);
                    ui.horizontal(|ui| {
                        ui.label("Input:");
                        ui.label(RichText::new(meta.input_type).monospace());
                        ui.label("→ Output:");
                        ui.label(RichText::new(meta.output_type).monospace());
                    });

                    // Key parameters
                    if !meta.key_parameters.is_empty() {
                        ui.add_space(5.0);
                        ui.label(RichText::new("Key Parameters:").strong());
                        for param in meta.key_parameters.iter() {
                            ui.label(format!("  • {}", param));
                        }
                    }
                });

            // Collapsible formulas section
            if !meta.formulas.is_empty() {
                egui::CollapsingHeader::new(RichText::new("📐 Formulas").strong())
                    .default_open(false)
                    .show(ui, |ui| {
                        for formula in meta.formulas.iter() {
                            ui.add_space(5.0);
                            ui.label(RichText::new(formula.name).strong());
                            ui.label(RichText::new(formula.plaintext).monospace());

                            // Show variables
                            if !formula.variables.is_empty() {
                                ui.indent("formula_vars", |ui| {
                                    for (var, desc) in formula.variables.iter() {
                                        ui.horizontal(|ui| {
                                            ui.label(RichText::new(*var).monospace());
                                            ui.label("=");
                                            ui.label(*desc);
                                        });
                                    }
                                });
                            }
                        }
                    });
            }

            // Collapsible implementation section
            if meta.implementation.is_some() || !meta.related_code.is_empty() {
                egui::CollapsingHeader::new(RichText::new("🔧 Implementation").strong())
                    .default_open(false)
                    .show(ui, |ui| {
                        if let Some(ref impl_loc) = meta.implementation {
                            ui.horizontal(|ui| {
                                ui.label("Primary:");
                                let code_path = format!("crates/{}/{}:{}",
                                    impl_loc.crate_name, impl_loc.file_path, impl_loc.line);
                                if ui.link(RichText::new(&code_path).monospace()).clicked() {
                                    self.open_code_location(impl_loc);
                                }
                            });
                            ui.label(format!("  {} → {}", impl_loc.symbol, impl_loc.description));
                        }

                        for code_loc in meta.related_code.iter() {
                            ui.horizontal(|ui| {
                                ui.label("Related:");
                                let code_path = format!("crates/{}/{}:{}",
                                    code_loc.crate_name, code_loc.file_path, code_loc.line);
                                if ui.link(RichText::new(&code_path).monospace()).clicked() {
                                    self.open_code_location(code_loc);
                                }
                            });
                        }
                    });
            }

            // Collapsible tests section
            if !meta.tests.is_empty() {
                egui::CollapsingHeader::new(RichText::new("🧪 Tests").strong())
                    .default_open(false)
                    .show(ui, |ui| {
                        for test in meta.tests.iter() {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new(test.name).monospace());
                                if ui.small_button("Run").clicked() {
                                    self.run_test(test);
                                }
                            });
                            ui.indent("test_desc", |ui| {
                                ui.label(test.description);
                                ui.label(format!("~{} ms", test.expected_runtime_ms));
                            });
                        }

                        ui.add_space(5.0);
                        if ui.button("Run All Tests").clicked() {
                            for test in meta.tests.iter() {
                                self.run_test(test);
                            }
                        }
                    });
            }

            // Collapsible performance section
            if let Some(ref perf) = meta.performance {
                egui::CollapsingHeader::new(RichText::new("⚡ Performance").strong())
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Complexity:");
                            ui.label(RichText::new(perf.complexity).monospace());
                        });
                        ui.horizontal(|ui| {
                            ui.label("Memory:");
                            ui.label(perf.memory);
                        });
                        ui.horizontal(|ui| {
                            if perf.simd_optimized {
                                ui.label(RichText::new("✓ SIMD").color(Color32::GREEN));
                            } else {
                                ui.label(RichText::new("✗ SIMD").color(Color32::GRAY));
                            }
                            if perf.gpu_accelerable {
                                ui.label(RichText::new("✓ GPU").color(Color32::GREEN));
                            } else {
                                ui.label(RichText::new("✗ GPU").color(Color32::GRAY));
                            }
                        });
                    });
            }

            // Collapsible standards section
            if !meta.standards.is_empty() {
                egui::CollapsingHeader::new(RichText::new("📋 Standards").strong())
                    .default_open(false)
                    .show(ui, |ui| {
                        for std_ref in meta.standards.iter() {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new(std_ref.name).strong());
                                if let Some(section) = std_ref.section {
                                    ui.label(section);
                                }
                            });
                            if let Some(url) = std_ref.url {
                                if ui.link("View specification").clicked() {
                                    #[cfg(not(target_arch = "wasm32"))]
                                    let _ = open::that(url);
                                }
                            }
                        }
                    });
            }

            // Related blocks
            if !meta.related_blocks.is_empty() {
                ui.add_space(5.0);
                ui.horizontal(|ui| {
                    ui.label("Related:");
                    for related in meta.related_blocks.iter() {
                        ui.label(RichText::new(*related).small().color(Color32::LIGHT_BLUE));
                    }
                });
            }
        } else {
            // No metadata available
            ui.add_space(10.0);
            ui.label(RichText::new("No documentation available for this block type.")
                .italics().color(Color32::GRAY));
        }
    }

    /// Open code location in editor
    fn open_code_location(&self, loc: &block_metadata::CodeLocation) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            // Construct the file path relative to project root
            let file_path = format!("crates/{}/{}", loc.crate_name, loc.file_path);

            // Try to open with VS Code at specific line
            let vscode_result = std::process::Command::new("code")
                .arg("--goto")
                .arg(format!("{}:{}", file_path, loc.line))
                .spawn();

            if vscode_result.is_err() {
                // Fallback: try to open the file with default handler
                log::info!("Opening file: {}:{}", file_path, loc.line);
                let _ = open::that(&file_path);
            }
        }
    }

    /// Run a block test
    fn run_test(&self, test: &block_metadata::BlockTest) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let cmd = block_metadata::get_test_command(test);
            log::info!("Running test: {}", cmd);

            // Run the test in background
            let parts: Vec<&str> = cmd.split_whitespace().collect();
            if parts.len() >= 2 {
                match std::process::Command::new(parts[0])
                    .args(&parts[1..])
                    .spawn()
                {
                    Ok(_) => log::info!("Test started: {}", test.name),
                    Err(e) => log::error!("Failed to run test: {}", e),
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::Pos2;

    #[test]
    fn test_cycle_detection_no_cycle() {
        let mut pipeline = Pipeline::new();

        // Create A -> B -> C (linear, no cycle)
        let a = pipeline.add_block(BlockType::BitSource { pattern: "random".to_string() }, Pos2::ZERO);
        let b = pipeline.add_block(BlockType::Scrambler { polynomial: 0x1D, seed: 0xFF }, Pos2::new(100.0, 0.0));
        let c = pipeline.add_block(BlockType::PskModulator { order: 2 }, Pos2::new(200.0, 0.0));

        pipeline.connect(a, 0, b, 0);
        pipeline.connect(b, 0, c, 0);

        // Adding C -> D should not create a cycle
        let d = pipeline.add_block(BlockType::PskModulator { order: 4 }, Pos2::new(300.0, 0.0));
        assert!(!pipeline.would_create_cycle(c, d));

        // Pipeline has no cycle
        assert!(!pipeline.has_cycle());
    }

    #[test]
    fn test_cycle_detection_simple_cycle() {
        let mut pipeline = Pipeline::new();

        // Create A -> B -> C
        let a = pipeline.add_block(BlockType::BitSource { pattern: "random".to_string() }, Pos2::ZERO);
        let b = pipeline.add_block(BlockType::Scrambler { polynomial: 0x1D, seed: 0xFF }, Pos2::new(100.0, 0.0));
        let c = pipeline.add_block(BlockType::PskModulator { order: 2 }, Pos2::new(200.0, 0.0));

        pipeline.connect(a, 0, b, 0);
        pipeline.connect(b, 0, c, 0);

        // Adding C -> A would create a cycle
        assert!(pipeline.would_create_cycle(c, a));

        // Adding C -> B would create a cycle
        assert!(pipeline.would_create_cycle(c, b));
    }

    #[test]
    fn test_cycle_detection_self_loop() {
        let mut pipeline = Pipeline::new();

        let a = pipeline.add_block(BlockType::BitSource { pattern: "random".to_string() }, Pos2::ZERO);

        // Self-loop is always a cycle
        assert!(pipeline.would_create_cycle(a, a));
    }

    #[test]
    fn test_cycle_detection_complex_graph() {
        let mut pipeline = Pipeline::new();

        // Create diamond pattern: A -> B, A -> C, B -> D, C -> D
        let a = pipeline.add_block(BlockType::BitSource { pattern: "random".to_string() }, Pos2::ZERO);
        let b = pipeline.add_block(BlockType::Scrambler { polynomial: 0x1D, seed: 0xFF }, Pos2::new(100.0, -50.0));
        let c = pipeline.add_block(BlockType::Scrambler { polynomial: 0x1D, seed: 0xAA }, Pos2::new(100.0, 50.0));
        let d = pipeline.add_block(BlockType::PskModulator { order: 2 }, Pos2::new(200.0, 0.0));

        pipeline.connect(a, 0, b, 0);
        pipeline.connect(a, 0, c, 0);
        pipeline.connect(b, 0, d, 0);
        pipeline.connect(c, 0, d, 1);

        // No cycle exists
        assert!(!pipeline.has_cycle());

        // D -> A would create a cycle
        assert!(pipeline.would_create_cycle(d, a));

        // D -> B would create a cycle
        assert!(pipeline.would_create_cycle(d, b));

        // D -> C would create a cycle
        assert!(pipeline.would_create_cycle(d, c));

        // New node E: D -> E should not create a cycle
        let e = pipeline.add_block(BlockType::PskModulator { order: 4 }, Pos2::new(300.0, 0.0));
        assert!(!pipeline.would_create_cycle(d, e));
    }
}
