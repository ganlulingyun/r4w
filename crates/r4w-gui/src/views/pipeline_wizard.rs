//! Pipeline-Based Waveform Builder
//!
//! A visual signal processing pipeline builder for creating waveform specifications.
//! Supports multiple blocks, parallel branches, and flexible routing.

use egui::{Ui, RichText, Color32, Pos2, Vec2, Rect, Stroke, epaint::PathShape};
use std::collections::HashMap;

/// Unique identifier for a pipeline block
pub type BlockId = u32;

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
            BlockType::FecEncoder { code_type, rate } => {
                format!("      code_type: {:?}\n      rate: \"{}\"\n", code_type, rate)
            }
            BlockType::PskModulator { order } => format!("      order: {}\n", order),
            BlockType::QamModulator { order } => format!("      order: {}\n", order),
            BlockType::FirFilter { filter_type, cutoff_hz, num_taps } => {
                format!("      filter_type: {:?}\n      cutoff_hz: {}\n      num_taps: {}\n",
                    filter_type, cutoff_hz, num_taps)
            }
            BlockType::PulseShaper { shape, rolloff, span } => {
                format!("      shape: {:?}\n      rolloff: {}\n      span: {}\n", shape, rolloff, span)
            }
            BlockType::Upsampler { factor } => format!("      factor: {}\n", factor),
            BlockType::Downsampler { factor } => format!("      factor: {}\n", factor),
            BlockType::AwgnChannel { snr_db } => format!("      snr_db: {}\n", snr_db),
            _ => String::new(),
        }
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
    pub dragging_block: Option<BlockType>,
    pub connecting_from: Option<(BlockId, u32)>,
    pub canvas_offset: Vec2,
    pub zoom: f32,
    pub show_library: bool,
    pub show_properties: bool,
    pub yaml_output: String,
}

impl Default for PipelineWizardView {
    fn default() -> Self {
        Self {
            pipeline: Pipeline::new(),
            selected_block: None,
            dragging_block: None,
            connecting_from: None,
            canvas_offset: Vec2::ZERO,
            zoom: 1.0,
            show_library: true,
            show_properties: true,
            yaml_output: String::new(),
        }
    }
}

impl PipelineWizardView {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn render(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.heading("Pipeline Waveform Builder");
            ui.add_space(20.0);
            ui.checkbox(&mut self.show_library, "Library");
            ui.checkbox(&mut self.show_properties, "Properties");

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("Export YAML").clicked() {
                    self.yaml_output = self.pipeline.to_yaml();
                }
                if ui.button("Clear").clicked() {
                    self.pipeline = Pipeline::new();
                    self.selected_block = None;
                }
            });
        });

        ui.separator();

        // Main layout: Library | Canvas | Properties
        ui.horizontal(|ui| {
            // Block library panel
            if self.show_library {
                ui.vertical(|ui| {
                    ui.set_width(180.0);
                    self.render_library(ui);
                });
                ui.separator();
            }

            // Main canvas
            ui.vertical(|ui| {
                self.render_canvas(ui);
            });

            // Properties panel
            if self.show_properties {
                ui.separator();
                ui.vertical(|ui| {
                    ui.set_width(250.0);
                    self.render_properties(ui);
                });
            }
        });

        // YAML output window
        if !self.yaml_output.is_empty() {
            egui::Window::new("Pipeline YAML")
                .resizable(true)
                .default_width(500.0)
                .show(ui.ctx(), |ui| {
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

    fn render_library(&mut self, ui: &mut Ui) {
        ui.label(RichText::new("Block Library").strong());
        ui.add_space(5.0);

        egui::ScrollArea::vertical().show(ui, |ui| {
            for (category, templates) in get_block_templates() {
                ui.collapsing(RichText::new(category.name()).color(category.color()), |ui| {
                    for template in templates {
                        let response = ui.add(
                            egui::Button::new(template.name())
                                .fill(category.color().gamma_multiply(0.3))
                        );

                        if response.clicked() {
                            // Add block to center of canvas
                            let pos = Pos2::new(300.0, 200.0) - self.canvas_offset;
                            self.pipeline.add_block(template.clone(), pos);
                        }

                        response.on_hover_text(format!("Click to add {} to pipeline", template.name()));
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

        // Background
        painter.rect_filled(rect, 0.0, Color32::from_rgb(30, 30, 40));

        // Grid
        let grid_spacing = 20.0 * self.zoom;
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
        for conn in &self.pipeline.connections {
            if let (Some(from_block), Some(to_block)) = (
                self.pipeline.blocks.get(&conn.from_block),
                self.pipeline.blocks.get(&conn.to_block),
            ) {
                let from_pos = self.block_output_pos(from_block, conn.from_port, rect);
                let to_pos = self.block_input_pos(to_block, conn.to_port, rect);

                // Bezier curve
                let ctrl1 = Pos2::new(from_pos.x + 50.0, from_pos.y);
                let ctrl2 = Pos2::new(to_pos.x - 50.0, to_pos.y);

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

                painter.add(PathShape::line(points, Stroke::new(2.0, Color32::from_rgb(100, 200, 100))));
            }
        }

        // Draw blocks
        let block_ids: Vec<_> = self.pipeline.blocks.keys().cloned().collect();
        for id in block_ids {
            if let Some(block) = self.pipeline.blocks.get(&id) {
                self.draw_block(&painter, block, rect);
            }
        }

        // Handle interactions
        if response.dragged() && response.drag_delta().length() > 0.0 {
            if let Some(selected_id) = self.selected_block {
                if let Some(block) = self.pipeline.blocks.get_mut(&selected_id) {
                    block.position += response.drag_delta() / self.zoom;
                }
            } else {
                self.canvas_offset += response.drag_delta();
            }
        }

        // Click to select
        if response.clicked() {
            let click_pos = response.interact_pointer_pos().unwrap_or(Pos2::ZERO);
            self.selected_block = None;

            for (id, block) in &self.pipeline.blocks {
                let block_rect = self.block_rect(block, rect);
                if block_rect.contains(click_pos) {
                    self.selected_block = Some(*id);
                    break;
                }
            }
        }

        // Delete selected block
        if self.selected_block.is_some() {
            ui.input(|i| {
                if i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace) {
                    if let Some(id) = self.selected_block.take() {
                        self.pipeline.remove_block(id);
                    }
                }
            });
        }

        // Zoom with scroll
        ui.input(|i| {
            let scroll = i.raw_scroll_delta.y;
            if scroll != 0.0 {
                self.zoom = (self.zoom + scroll * 0.001).clamp(0.5, 2.0);
            }
        });

        // Instructions
        painter.text(
            rect.left_bottom() + Vec2::new(10.0, -10.0),
            egui::Align2::LEFT_BOTTOM,
            "Click blocks in library to add | Drag to move | Delete to remove | Scroll to zoom",
            egui::FontId::proportional(12.0),
            Color32::GRAY,
        );
    }

    fn draw_block(&self, painter: &egui::Painter, block: &PipelineBlock, canvas_rect: Rect) {
        let block_rect = self.block_rect(block, canvas_rect);
        let category = block.block_type.category();
        let is_selected = self.selected_block == Some(block.id);

        // Block background
        let bg_color = if is_selected {
            category.color().gamma_multiply(0.6)
        } else {
            category.color().gamma_multiply(0.4)
        };
        painter.rect_filled(block_rect, 5.0, bg_color);

        // Border
        let border_color = if is_selected {
            Color32::WHITE
        } else {
            category.color()
        };
        painter.rect_stroke(block_rect, 5.0, Stroke::new(2.0, border_color));

        // Block name
        painter.text(
            block_rect.center(),
            egui::Align2::CENTER_CENTER,
            &block.name,
            egui::FontId::proportional(12.0 * self.zoom),
            Color32::WHITE,
        );

        // Input ports
        for i in 0..block.block_type.num_inputs() {
            let port_pos = self.block_input_pos(block, i, canvas_rect);
            painter.circle_filled(port_pos, 5.0 * self.zoom, Color32::from_rgb(100, 100, 200));
        }

        // Output ports
        for i in 0..block.block_type.num_outputs() {
            let port_pos = self.block_output_pos(block, i, canvas_rect);
            painter.circle_filled(port_pos, 5.0 * self.zoom, Color32::from_rgb(200, 100, 100));
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

    fn render_properties(&mut self, ui: &mut Ui) {
        ui.label(RichText::new("Properties").strong());
        ui.add_space(5.0);

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
                        });
                });
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::QamModulator { order: new_order };
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
                });

                ui.horizontal(|ui| {
                    ui.label("Cutoff (Hz):");
                    ui.add(egui::DragValue::new(&mut new_cutoff).speed(100.0));
                });

                ui.horizontal(|ui| {
                    ui.label("Taps:");
                    ui.add(egui::DragValue::new(&mut new_taps).range(8..=256));
                });

                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::FirFilter {
                        filter_type: new_type,
                        cutoff_hz: new_cutoff,
                        num_taps: new_taps
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
                    ui.radio_value(&mut new_shape, PulseShape::Gaussian, "Gauss");
                });

                ui.horizontal(|ui| {
                    ui.label("Rolloff:");
                    ui.add(egui::Slider::new(&mut new_rolloff, 0.0..=1.0));
                });

                ui.horizontal(|ui| {
                    ui.label("Span:");
                    ui.add(egui::DragValue::new(&mut new_span).range(4..=16));
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
                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::Downsampler { factor: new_factor };
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
            BlockType::CssModulator { sf, bw_hz } => {
                let mut new_sf = sf;
                let mut new_bw = bw_hz;

                ui.horizontal(|ui| {
                    ui.label("SF:");
                    ui.add(egui::DragValue::new(&mut new_sf).range(5..=12));
                });

                ui.horizontal(|ui| {
                    ui.label("BW (Hz):");
                    egui::ComboBox::from_id_salt("css_bw")
                        .selected_text(format!("{} kHz", new_bw / 1000))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut new_bw, 125000, "125 kHz");
                            ui.selectable_value(&mut new_bw, 250000, "250 kHz");
                            ui.selectable_value(&mut new_bw, 500000, "500 kHz");
                        });
                });

                if let Some(block) = self.pipeline.blocks.get_mut(&block_id) {
                    block.block_type = BlockType::CssModulator { sf: new_sf, bw_hz: new_bw };
                }
            }
            _ => {
                ui.label("(No editable parameters)");
            }
        }
    }
}
