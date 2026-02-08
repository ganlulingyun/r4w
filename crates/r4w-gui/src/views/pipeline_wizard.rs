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
use std::collections::{HashMap, HashSet};

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
#[derive(Clone, Debug, PartialEq)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FecType {
    Convolutional,
    Turbo,
    Ldpc,
    ReedSolomon,
    Hamming,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CrcType {
    Crc8,
    Crc16Ccitt,
    Crc16Ibm,
    Crc32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstellationType {
    Bpsk,
    Qpsk,
    Psk8,
    Qam16,
    Qam64,
    Qam256,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterType {
    Lowpass,
    Highpass,
    Bandpass,
    Bandstop,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IirDesign {
    Butterworth,
    Chebyshev1,
    Chebyshev2,
    Bessel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PulseShape {
    RootRaisedCosine,
    RaisedCosine,
    Gaussian,
    Rectangular,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FadingModel {
    Rayleigh,
    Rician,
    Nakagami,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AgcMode {
    Fast,
    Slow,
    Adaptive,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimingAlgo {
    EarlyLate,
    Gardner,
    MuellerMuller,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CarrierAlgo {
    CostasLoop,
    DecisionDirected,
    PilotAided,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EqualizerType {
    Lms,
    Rls,
    Cma,
    Dfe,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutputFormat {
    ComplexFloat32,
    ComplexFloat64,
    ComplexInt16,
    ComplexInt8,
}

/// A block in the pipeline
#[derive(Clone, Debug)]
pub struct PipelineBlock {
    pub id: BlockId,
    pub block_type: BlockType,
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
#[derive(Clone, Debug)]
pub struct Connection {
    pub from_block: BlockId,
    pub from_port: u32,
    pub to_block: BlockId,
    pub to_port: u32,
}

/// The complete pipeline
#[derive(Clone, Debug, Default)]
pub struct Pipeline {
    pub name: String,
    pub description: String,
    pub blocks: HashMap<BlockId, PipelineBlock>,
    pub connections: Vec<Connection>,
    next_id: BlockId,
}

impl Pipeline {
    pub fn new() -> Self {
        Self {
            name: "New Pipeline".to_string(),
            description: String::new(),
            blocks: HashMap::new(),
            connections: Vec::new(),
            next_id: 1,
        }
    }

    pub fn add_block(&mut self, block_type: BlockType, position: Pos2) -> BlockId {
        let id = self.next_id;
        self.next_id += 1;
        let block = PipelineBlock::new(id, block_type, position);
        self.blocks.insert(id, block);
        id
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

        // Position blocks
        let block_width = 140.0;
        let block_height = 70.0;
        let h_spacing = 30.0;
        let v_spacing = 20.0;

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

    /// Simple grid layout for disconnected blocks
    fn grid_layout(&mut self) {
        let mut block_ids: Vec<_> = self.blocks.keys().cloned().collect();
        block_ids.sort();

        let cols = 4;
        let block_width = 150.0;
        let block_height = 80.0;

        for (idx, id) in block_ids.iter().enumerate() {
            let col = idx % cols;
            let row = idx / cols;
            if let Some(block) = self.blocks.get_mut(id) {
                block.position = Pos2::new(
                    50.0 + col as f32 * block_width,
                    50.0 + row as f32 * block_height,
                );
            }
        }
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
    pub selected_block: Option<BlockId>,
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
}

impl Default for PipelineWizardView {
    fn default() -> Self {
        Self {
            pipeline: Pipeline::new(),
            selected_block: None,
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
        }
    }
}

impl PipelineWizardView {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn render(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();

        // Top panel: Toolbar (rendered first so side panels appear below it)
        egui::TopBottomPanel::top("pipeline_toolbar")
            .show(&ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.heading("Pipeline Waveform Builder");
                    ui.add_space(20.0);
                    ui.checkbox(&mut self.show_library, "Library");
                    ui.checkbox(&mut self.show_properties, "Properties");
                    ui.checkbox(&mut self.snap_to_grid, "Snap");

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("Export YAML").clicked() {
                            self.yaml_output = self.pipeline.to_yaml();
                        }
                        if ui.button("Validate").clicked() {
                            self.validation_result = self.pipeline.validate();
                            self.show_validation = true;
                        }
                        if ui.button("Auto-Layout").clicked() {
                            self.pipeline.auto_layout();
                        }
                        if ui.button("Presets").clicked() {
                            self.show_presets = !self.show_presets;
                        }
                        if ui.button("Clear").clicked() {
                            self.pipeline = Pipeline::new();
                            self.selected_block = None;
                            self.selected_connection = None;
                            self.connecting_from = None;
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
                                self.selected_block = None;
                                self.selected_connection = None;
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
                    self.render_properties_content(ui);
                });
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
                                // Add block to center of canvas
                                let pos = Pos2::new(300.0, 200.0) - self.canvas_offset;
                                self.pipeline.add_block(template.clone(), pos);
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

        // Draw connections
        for (idx, conn) in self.pipeline.connections.iter().enumerate() {
            if let (Some(from_block), Some(to_block)) = (
                self.pipeline.blocks.get(&conn.from_block),
                self.pipeline.blocks.get(&conn.to_block),
            ) {
                let from_pos = self.block_output_pos(from_block, conn.from_port, rect);
                let to_pos = self.block_input_pos(to_block, conn.to_port, rect);

                let is_selected = self.selected_connection == Some(idx);
                let color = if is_selected {
                    Color32::from_rgb(255, 200, 100)
                } else {
                    Color32::from_rgb(100, 200, 100)
                };
                let width = if is_selected { 3.0 } else { 2.0 };

                self.draw_bezier(&painter, from_pos, to_pos, color, width);
            }
        }

        // Draw in-progress connection
        if let Some((from_block_id, from_port)) = self.connecting_from {
            if let Some(from_block) = self.pipeline.blocks.get(&from_block_id) {
                let from_pos = self.block_output_pos(from_block, from_port, rect);
                let to_pos = pointer_pos.unwrap_or(from_pos);

                self.draw_bezier(&painter, from_pos, to_pos, Color32::from_rgb(200, 200, 100), 2.0);

                // Show instruction
                painter.text(
                    rect.center_top() + Vec2::new(0.0, 30.0),
                    egui::Align2::CENTER_CENTER,
                    "Click on an input port to complete connection, or press ESC to cancel",
                    egui::FontId::proportional(14.0),
                    Color32::YELLOW,
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

        // Handle port clicks for connection creation
        if response.clicked() {
            if let Some(click_pos) = pointer_pos {
                // Check if clicking on a port
                let mut port_clicked = false;
                let mut connection_to_make: Option<(BlockId, u32, BlockId, u32)> = None;

                // First check if we're finishing a connection
                if self.connecting_from.is_some() {
                    // Collect port info first to avoid borrowing conflict
                    let port_info: Vec<_> = self.pipeline.blocks.iter()
                        .flat_map(|(id, block)| {
                            (0..block.block_type.num_inputs())
                                .map(|port| (*id, port, self.block_input_pos(block, port, rect)))
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
                    // Collect port info first
                    let port_info: Vec<_> = self.pipeline.blocks.iter()
                        .flat_map(|(id, block)| {
                            (0..block.block_type.num_outputs())
                                .map(|port| (*id, port, self.block_output_pos(block, port, rect)))
                                .collect::<Vec<_>>()
                        })
                        .collect();

                    for (id, port, port_pos) in port_info {
                        if (click_pos - port_pos).length() < 10.0 * self.zoom {
                            self.connecting_from = Some((id, port));
                            self.selected_block = None;
                            self.selected_connection = None;
                            port_clicked = true;
                            break;
                        }
                    }
                }

                // Now make the connection if needed
                if let Some((from_block, from_port, to_block, to_port)) = connection_to_make {
                    self.pipeline.connect(from_block, from_port, to_block, to_port);
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
                        self.selected_block = None;
                    } else {
                        // Check if clicking on a block
                        self.selected_block = None;
                        self.selected_connection = None;

                        for (id, block) in &self.pipeline.blocks {
                            let block_rect = self.block_rect(block, rect);
                            if block_rect.contains(click_pos) {
                                self.selected_block = Some(*id);
                                break;
                            }
                        }
                    }
                }
            }
        }

        // Handle dragging
        if response.dragged() && response.drag_delta().length() > 0.0 {
            if self.connecting_from.is_none() {
                if let Some(selected_id) = self.selected_block {
                    if let Some(block) = self.pipeline.blocks.get_mut(&selected_id) {
                        let mut new_pos = block.position + response.drag_delta() / self.zoom;
                        if self.snap_to_grid {
                            new_pos.x = (new_pos.x / self.grid_size).round() * self.grid_size;
                            new_pos.y = (new_pos.y / self.grid_size).round() * self.grid_size;
                        }
                        block.position = new_pos;
                    }
                } else {
                    self.canvas_offset += response.drag_delta();
                }
            }
        }

        // Keyboard shortcuts
        ui.input(|i| {
            // ESC to cancel connection
            if i.key_pressed(egui::Key::Escape) {
                self.connecting_from = None;
                self.selected_block = None;
                self.selected_connection = None;
            }

            // Delete selected block or connection
            if i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace) {
                if let Some(id) = self.selected_block.take() {
                    self.pipeline.remove_block(id);
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
            if let Some(id) = self.selected_block {
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
                    self.selected_block = None;
                    ui.close_menu();
                }
                if ui.button("Disconnect All").clicked() {
                    self.pipeline.connections.retain(|c| c.from_block != id && c.to_block != id);
                    ui.close_menu();
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
            "MODE: Connecting..."
        } else if self.selected_block.is_some() {
            "Selected: Block"
        } else if self.selected_connection.is_some() {
            "Selected: Connection"
        } else {
            ""
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
            "Click output ports to connect | Drag blocks to move | Delete/Backspace to remove | Scroll to zoom | Right-click for menu",
            egui::FontId::proportional(11.0),
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
        let is_selected = self.selected_block == Some(block.id);
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

        // Input ports with hover effect
        let port_radius = 6.0 * self.zoom;
        for i in 0..block.block_type.num_inputs() {
            let port_pos = self.block_input_pos(block, i, canvas_rect);

            // Check if this is a valid connection target
            let is_connection_target = self.connecting_from.is_some();
            let port_color = if is_connection_target {
                Color32::from_rgb(100, 200, 255) // Highlight when connecting
            } else {
                Color32::from_rgb(80, 120, 200)
            };

            // Port outer ring
            painter.circle_stroke(port_pos, port_radius + 2.0, Stroke::new(1.0, port_color));
            // Port fill
            painter.circle_filled(port_pos, port_radius, port_color);
            // Port inner highlight
            painter.circle_filled(port_pos, port_radius * 0.4, Color32::WHITE);
        }

        // Output ports
        for i in 0..block.block_type.num_outputs() {
            let port_pos = self.block_output_pos(block, i, canvas_rect);
            let is_connection_source = self.connecting_from == Some((block.id, i));
            let port_color = if is_connection_source {
                Color32::from_rgb(255, 200, 100) // Highlight when this is source
            } else {
                Color32::from_rgb(200, 80, 80)
            };

            // Port outer ring
            painter.circle_stroke(port_pos, port_radius + 2.0, Stroke::new(1.0, port_color));
            // Port fill
            painter.circle_filled(port_pos, port_radius, port_color);
            // Port inner highlight
            painter.circle_filled(port_pos, port_radius * 0.4, Color32::WHITE);
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

    fn render_properties_content(&mut self, ui: &mut Ui) {
        if let Some(id) = self.selected_block {
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

                ui.add_space(10.0);
                if ui.button("Delete Block").clicked() {
                    self.pipeline.remove_block(id);
                    self.selected_block = None;
                }
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
}
