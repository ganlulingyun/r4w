//! Waveform Wizard - Interactive waveform specification builder
//!
//! Guides users through creating a waveform specification YAML file
//! using the R4W waveform-spec schema.
//!
//! trace:FR-0087 | ai:claude

use egui::{Ui, RichText, Color32};

/// Wizard step enumeration
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WizardStep {
    Identity,
    Modulation,
    SpreadSpectrum,
    PulseShaping,
    Timing,
    Coding,
    Spectral,
    Review,
}

impl WizardStep {
    fn label(&self) -> &'static str {
        match self {
            Self::Identity => "1. Identity",
            Self::Modulation => "2. Modulation",
            Self::SpreadSpectrum => "3. Spread Spectrum",
            Self::PulseShaping => "4. Pulse Shaping",
            Self::Timing => "5. Timing",
            Self::Coding => "6. Channel Coding",
            Self::Spectral => "7. Spectral",
            Self::Review => "8. Review & Export",
        }
    }

    fn next(&self) -> Option<Self> {
        match self {
            Self::Identity => Some(Self::Modulation),
            Self::Modulation => Some(Self::SpreadSpectrum),
            Self::SpreadSpectrum => Some(Self::PulseShaping),
            Self::PulseShaping => Some(Self::Timing),
            Self::Timing => Some(Self::Coding),
            Self::Coding => Some(Self::Spectral),
            Self::Spectral => Some(Self::Review),
            Self::Review => None,
        }
    }

    fn prev(&self) -> Option<Self> {
        match self {
            Self::Identity => None,
            Self::Modulation => Some(Self::Identity),
            Self::SpreadSpectrum => Some(Self::Modulation),
            Self::PulseShaping => Some(Self::SpreadSpectrum),
            Self::Timing => Some(Self::PulseShaping),
            Self::Coding => Some(Self::Timing),
            Self::Spectral => Some(Self::Coding),
            Self::Review => Some(Self::Spectral),
        }
    }
}

/// Waveform type classification
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaveformType {
    Digital,
    Analog,
    Hybrid,
}

/// Modulation domain
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub enum ModulationDomain {
    Amplitude,
    Frequency,
    Phase,
    Hybrid,
}

/// Digital modulation scheme
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub enum DigitalScheme {
    Bpsk,
    Qpsk,
    Pi4Qpsk,
    Oqpsk,
    Qam16,
    Qam64,
    Fsk2,
    Fsk4,
    Msk,
    Gmsk,
    Ook,
    Css,
}

impl DigitalScheme {
    fn order(&self) -> u32 {
        match self {
            Self::Bpsk | Self::Fsk2 | Self::Msk | Self::Gmsk | Self::Ook => 2,
            Self::Qpsk | Self::Pi4Qpsk | Self::Oqpsk | Self::Fsk4 => 4,
            Self::Qam16 => 16,
            Self::Qam64 => 64,
            Self::Css => 128, // SF7
        }
    }
}

/// Analog modulation type
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub enum AnalogScheme {
    Am,
    DsbSc,
    SsbUsb,
    SsbLsb,
    Nbfm,
    Wbfm,
    Pm,
}

/// Spread spectrum technique
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub enum SpreadTechnique {
    None,
    Dsss,
    Fhss,
    Thss,
    Css,
    Hybrid,
}

/// PN sequence type
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub enum PnSequenceType {
    Lfsr,
    Gold,
    Kasami,
    Barker,
    Custom,
}

/// Pulse shaping filter type
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilterType {
    Rectangular,
    RaisedCosine,
    RootRaisedCosine,
    Gaussian,
}

/// FEC type
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub enum FecType {
    None,
    Hamming,
    Convolutional,
    Turbo,
    Ldpc,
}

/// Waveform preset templates
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaveformPreset {
    Custom,
    AmBroadcast,
    FmBroadcast,
    Bpsk,
    Qpsk,
    LoRaSF7,
    LoRaSF12,
    DsssGold,
    Fhss,
    Gmsk,
    Stanag4285,
}

impl WaveformPreset {
    fn label(&self) -> &'static str {
        match self {
            Self::Custom => "Custom (Start from scratch)",
            Self::AmBroadcast => "AM Broadcast (DSB-FC)",
            Self::FmBroadcast => "FM Broadcast (WBFM)",
            Self::Bpsk => "BPSK (Basic PSK)",
            Self::Qpsk => "QPSK (Quaternary PSK)",
            Self::LoRaSF7 => "LoRa SF7 (CSS)",
            Self::LoRaSF12 => "LoRa SF12 (Long Range)",
            Self::DsssGold => "DSSS with Gold Codes",
            Self::Fhss => "Frequency Hopping",
            Self::Gmsk => "GMSK (GSM-style)",
            Self::Stanag4285 => "STANAG-4285 (HF Modem)",
        }
    }

    fn description(&self) -> &'static str {
        match self {
            Self::Custom => "Start with default values and customize everything",
            Self::AmBroadcast => "Standard AM radio, envelope detection, 5 kHz audio",
            Self::FmBroadcast => "Wideband FM, 75 kHz deviation, stereo capable",
            Self::Bpsk => "Binary PSK, 1 bit/symbol, simple and robust",
            Self::Qpsk => "Quaternary PSK, 2 bits/symbol, common in satcom",
            Self::LoRaSF7 => "Chirp spread spectrum, SF7, high data rate",
            Self::LoRaSF12 => "Chirp spread spectrum, SF12, maximum range",
            Self::DsssGold => "Direct sequence with Gold codes, LPD/LPI capable",
            Self::Fhss => "Frequency hopping, jam resistant, military style",
            Self::Gmsk => "Gaussian MSK, constant envelope, spectral efficient",
            Self::Stanag4285 => "NATO HF data modem, 8-PSK, robust FEC",
        }
    }
}

/// Waveform specification state
#[derive(Clone)]
pub struct WaveformSpec {
    // Identity
    pub name: String,
    pub full_name: String,
    pub version: String,
    pub description: String,
    pub waveform_type: WaveformType,
    pub lpd_lpi_enabled: bool,
    pub target_processing_gain_db: f32,

    // Modulation
    pub domain: ModulationDomain,
    pub digital_scheme: DigitalScheme,
    pub analog_scheme: AnalogScheme,
    pub use_analog: bool,
    pub carrier_freq_hz: f64,
    pub am_modulation_index: f32,
    pub fm_deviation_hz: f32,

    // Spread spectrum
    pub spread_technique: SpreadTechnique,
    pub dsss_enabled: bool,
    pub chip_rate: u32,
    pub chips_per_symbol: u32,
    pub pn_type: PnSequenceType,
    pub pn_length: u32,
    pub fhss_enabled: bool,
    pub num_hop_channels: u32,
    pub hop_rate: u32,
    pub css_sf: u8,
    pub css_bandwidth_hz: u32,

    // Pulse shaping
    pub pulse_shaping_enabled: bool,
    pub filter_type: FilterType,
    pub rolloff: f32,
    pub span_symbols: u32,
    pub bt_product: f32,

    // Timing
    pub symbol_rate: u32,
    pub sample_rate: u32,
    pub burst_mode: bool,
    pub burst_duration_ms: f32,

    // Channel coding
    pub fec_enabled: bool,
    pub fec_type: FecType,
    pub code_rate: String,
    pub interleaving_enabled: bool,
    pub scrambling_enabled: bool,

    // Spectral
    pub bandwidth_hz: f32,
    pub target_psd_dbm_hz: f32,
}

impl Default for WaveformSpec {
    fn default() -> Self {
        Self {
            // Identity
            name: "MY-WAVEFORM".to_string(),
            full_name: "My Custom Waveform".to_string(),
            version: "1.0".to_string(),
            description: "A custom waveform specification.".to_string(),
            waveform_type: WaveformType::Digital,
            lpd_lpi_enabled: false,
            target_processing_gain_db: 20.0,

            // Modulation
            domain: ModulationDomain::Phase,
            digital_scheme: DigitalScheme::Qpsk,
            analog_scheme: AnalogScheme::Am,
            use_analog: false,
            carrier_freq_hz: 1_000_000.0,
            am_modulation_index: 0.8,
            fm_deviation_hz: 5000.0,

            // Spread spectrum
            spread_technique: SpreadTechnique::None,
            dsss_enabled: false,
            chip_rate: 1_000_000,
            chips_per_symbol: 127,
            pn_type: PnSequenceType::Gold,
            pn_length: 127,
            fhss_enabled: false,
            num_hop_channels: 50,
            hop_rate: 100,
            css_sf: 7,
            css_bandwidth_hz: 125_000,

            // Pulse shaping
            pulse_shaping_enabled: true,
            filter_type: FilterType::RootRaisedCosine,
            rolloff: 0.35,
            span_symbols: 8,
            bt_product: 0.3,

            // Timing
            symbol_rate: 1000,
            sample_rate: 48000,
            burst_mode: false,
            burst_duration_ms: 10.0,

            // Channel coding
            fec_enabled: true,
            fec_type: FecType::Convolutional,
            code_rate: "1/2".to_string(),
            interleaving_enabled: true,
            scrambling_enabled: true,

            // Spectral
            bandwidth_hz: 10000.0,
            target_psd_dbm_hz: -130.0,
        }
    }
}

impl WaveformSpec {
    /// Generate YAML output matching waveform-spec/schema.yaml
    pub fn to_yaml(&self) -> String {
        let wf_type = match self.waveform_type {
            WaveformType::Digital => "digital",
            WaveformType::Analog => "analog",
            WaveformType::Hybrid => "hybrid",
        };

        let category = match self.spread_technique {
            SpreadTechnique::None => "narrowband",
            SpreadTechnique::Dsss | SpreadTechnique::Fhss | SpreadTechnique::Thss |
            SpreadTechnique::Css | SpreadTechnique::Hybrid => "spread_spectrum",
        };

        let mod_domain = match self.domain {
            ModulationDomain::Amplitude => "amplitude",
            ModulationDomain::Frequency => "frequency",
            ModulationDomain::Phase => "phase",
            ModulationDomain::Hybrid => "hybrid",
        };

        let scheme = match self.digital_scheme {
            DigitalScheme::Bpsk => "BPSK",
            DigitalScheme::Qpsk => "QPSK",
            DigitalScheme::Pi4Qpsk => "PI4-QPSK",
            DigitalScheme::Oqpsk => "OQPSK",
            DigitalScheme::Qam16 => "16-QAM",
            DigitalScheme::Qam64 => "64-QAM",
            DigitalScheme::Fsk2 => "2-FSK",
            DigitalScheme::Fsk4 => "4-FSK",
            DigitalScheme::Msk => "MSK",
            DigitalScheme::Gmsk => "GMSK",
            DigitalScheme::Ook => "OOK",
            DigitalScheme::Css => "CSS",
        };

        let spread_tech = match self.spread_technique {
            SpreadTechnique::None => "none",
            SpreadTechnique::Dsss => "dsss",
            SpreadTechnique::Fhss => "fhss",
            SpreadTechnique::Thss => "thss",
            SpreadTechnique::Css => "css",
            SpreadTechnique::Hybrid => "hybrid",
        };

        let pn_type = match self.pn_type {
            PnSequenceType::Lfsr => "lfsr",
            PnSequenceType::Gold => "gold",
            PnSequenceType::Kasami => "kasami",
            PnSequenceType::Barker => "barker",
            PnSequenceType::Custom => "custom",
        };

        let filter = match self.filter_type {
            FilterType::Rectangular => "rectangular",
            FilterType::RaisedCosine => "raised_cosine",
            FilterType::RootRaisedCosine => "root_raised_cosine",
            FilterType::Gaussian => "gaussian",
        };

        let fec = match self.fec_type {
            FecType::None => "none",
            FecType::Hamming => "hamming",
            FecType::Convolutional => "convolutional",
            FecType::Turbo => "turbo",
            FecType::Ldpc => "ldpc",
        };

        let samples_per_symbol = if self.symbol_rate > 0 {
            self.sample_rate / self.symbol_rate
        } else {
            1
        };

        format!(
            r#"# Waveform Specification
# Generated by R4W Waveform Wizard
# Schema: waveform-spec/schema.yaml v1.0
---
# ============================================================================
# WAVEFORM IDENTITY
# ============================================================================
waveform:
  name: "{name}"
  full_name: "{full_name}"
  version: "{version}"
  description: |
    {description}

  classification:
    type: "{wf_type}"
    category: "{category}"

  lpd_lpi:
    enabled: {lpd_enabled}
    target_processing_gain_db: {target_pg}

# ============================================================================
# MODULATION
# ============================================================================
modulation:
  domain: "{mod_domain}"
  order: {order}
  scheme: "{scheme}"

  constellation:
    type: "gray"
    rotation_deg: 0

  differential:
    enabled: false
    reference: "previous"

  analog:
    enabled: {analog_enabled}
    carrier_freq_hz: {carrier_freq}

# ============================================================================
# SPREAD SPECTRUM
# ============================================================================
spread_spectrum:
  enabled: {spread_enabled}
  technique: "{spread_tech}"

  dsss:
    enabled: {dsss_enabled}
    chip_rate: {chip_rate}
    chips_per_symbol: {chips_per_symbol}
    pn_sequence:
      type: "{pn_type}"
      length: {pn_length}

  fhss:
    enabled: {fhss_enabled}
    num_channels: {num_hop_channels}
    hop_rate: {hop_rate}
    dwell_time_ms: {dwell_time}
    hop_pattern:
      type: "pseudo_random"

  thss:
    enabled: {thss_enabled}

  css:
    enabled: {css_enabled}
    spreading_factor: {css_sf}
    bandwidth_hz: {css_bw}

# ============================================================================
# PULSE SHAPING
# ============================================================================
pulse_shaping:
  enabled: {ps_enabled}
  filter:
    type: "{filter}"
    rolloff: {rolloff}
    span_symbols: {span_symbols}
    bt_product: {bt_product}

# ============================================================================
# TIMING
# ============================================================================
timing:
  symbol_rate: {symbol_rate}
  sample_rate: {sample_rate}
  samples_per_symbol: {samples_per_symbol}
  burst:
    enabled: {burst_enabled}
    burst_duration_ms: {burst_duration}

# ============================================================================
# CHANNEL CODING
# ============================================================================
channel_coding:
  enabled: {fec_enabled}
  fec:
    type: "{fec}"
    rate: "{code_rate}"
  interleaving:
    enabled: {interleaving}
    type: "block"
  scrambling:
    enabled: {scrambling}

# ============================================================================
# SPECTRAL CHARACTERISTICS
# ============================================================================
spectral:
  bandwidth:
    occupied_99_hz: {bandwidth}
  psd:
    target_dbm_hz: {target_psd}

# ============================================================================
# CALCULATED METRICS
# ============================================================================
metrics:
  processing_gain_db: {pg:.1}
  spectral_efficiency_bps_hz: {se:.4}
  data_rate_bps: {dr}
  chip_rate: {chip_rate_metric}
"#,
            name = self.name,
            full_name = self.full_name,
            version = self.version,
            description = self.description,
            wf_type = wf_type,
            category = category,
            lpd_enabled = self.lpd_lpi_enabled,
            target_pg = self.target_processing_gain_db,
            mod_domain = mod_domain,
            order = self.digital_scheme.order(),
            scheme = scheme,
            analog_enabled = self.use_analog,
            carrier_freq = self.carrier_freq_hz,
            spread_enabled = self.spread_technique != SpreadTechnique::None,
            spread_tech = spread_tech,
            dsss_enabled = self.dsss_enabled,
            chip_rate = self.chip_rate,
            chips_per_symbol = self.chips_per_symbol,
            pn_type = pn_type,
            pn_length = self.pn_length,
            fhss_enabled = self.fhss_enabled,
            num_hop_channels = self.num_hop_channels,
            hop_rate = self.hop_rate,
            dwell_time = if self.hop_rate > 0 { 1000.0 / self.hop_rate as f32 } else { 10.0 },
            thss_enabled = self.spread_technique == SpreadTechnique::Thss,
            css_enabled = self.spread_technique == SpreadTechnique::Css,
            css_sf = self.css_sf,
            css_bw = self.css_bandwidth_hz,
            ps_enabled = self.pulse_shaping_enabled,
            filter = filter,
            rolloff = self.rolloff,
            span_symbols = self.span_symbols,
            bt_product = self.bt_product,
            symbol_rate = self.symbol_rate,
            sample_rate = self.sample_rate,
            samples_per_symbol = samples_per_symbol,
            burst_enabled = self.burst_mode,
            burst_duration = self.burst_duration_ms,
            fec_enabled = self.fec_enabled,
            fec = fec,
            code_rate = self.code_rate,
            interleaving = self.interleaving_enabled,
            scrambling = self.scrambling_enabled,
            bandwidth = self.bandwidth_hz,
            target_psd = self.target_psd_dbm_hz,
            pg = self.compute_processing_gain(),
            se = self.compute_spectral_efficiency(),
            dr = self.compute_data_rate(),
            chip_rate_metric = if self.dsss_enabled { self.chip_rate } else { 0 },
        )
    }

    fn compute_processing_gain(&self) -> f32 {
        if self.dsss_enabled {
            10.0 * (self.chips_per_symbol as f32).log10()
        } else if self.spread_technique == SpreadTechnique::Css {
            self.css_sf as f32  // SF7 = ~7 dB processing gain
        } else {
            0.0
        }
    }

    fn compute_spectral_efficiency(&self) -> f32 {
        let bits_per_symbol = (self.digital_scheme.order() as f32).log2();
        if self.dsss_enabled {
            bits_per_symbol / self.chips_per_symbol as f32
        } else {
            bits_per_symbol
        }
    }

    fn compute_data_rate(&self) -> u32 {
        let bits_per_symbol = (self.digital_scheme.order() as f32).log2();
        let code_rate: f32 = match self.code_rate.as_str() {
            "1/2" => 0.5,
            "2/3" => 0.67,
            "3/4" => 0.75,
            "4/5" => 0.8,
            _ => 1.0,
        };
        (self.symbol_rate as f32 * bits_per_symbol * code_rate) as u32
    }
}

/// Waveform Wizard View
pub struct WaveformWizardView {
    current_step: WizardStep,
    selected_preset: WaveformPreset,
    spec: WaveformSpec,
    yaml_output: String,
    show_export_dialog: bool,
    include_implementation_prompt: bool,
}

impl Default for WaveformWizardView {
    fn default() -> Self {
        Self {
            current_step: WizardStep::Identity,
            selected_preset: WaveformPreset::Custom,
            spec: WaveformSpec::default(),
            yaml_output: String::new(),
            show_export_dialog: false,
            include_implementation_prompt: false,
        }
    }
}

impl WaveformWizardView {
    pub fn new() -> Self {
        Self::default()
    }

    /// Generate export content based on current settings
    fn generate_export_content(&self) -> String {
        let yaml = self.spec.to_yaml();

        if self.include_implementation_prompt {
            format!(
                r#"# R4W Waveform Implementation Prompt
#
# This document contains everything needed for an AI assistant to implement
# a new waveform in the R4W platform. Simply paste this entire content into
# your AI chat and ask for implementation.
#
# Generated by R4W Waveform Wizard

---

## R4W Platform Context

### Project Structure
```
crates/r4w-core/src/waveform/
  mod.rs          # Waveform trait and factory
  your_waveform.rs  # Create this file
```

### The Waveform Trait

Every waveform must implement this trait from `crates/r4w-core/src/waveform/mod.rs`:

```rust
pub trait Waveform: Debug + Send + Sync {{
    fn info(&self) -> WaveformInfo;
    fn common_params(&self) -> &CommonParams;
    fn modulate(&self, data: &[u8]) -> Vec<IQSample>;
    fn demodulate(&self, samples: &[IQSample]) -> DemodResult;
    fn samples_per_symbol(&self) -> usize;
    fn get_visualization(&self, data: &[u8]) -> VisualizationData;
}}
```

### Core Types

```rust
pub struct IQSample {{ pub re: f32, pub im: f32 }}

pub struct CommonParams {{
    pub sample_rate: f64,
    pub carrier_freq: f64,
    pub amplitude: f64,
}}

pub struct WaveformInfo {{
    pub name: &'static str,
    pub full_name: &'static str,
    pub description: &'static str,
    pub complexity: u8,
    pub bits_per_symbol: u8,
    pub carries_data: bool,
    pub characteristics: &'static [&'static str],
    pub history: &'static str,
    pub modern_usage: &'static str,
}}

pub struct DemodResult {{
    pub bits: Vec<u8>,
    pub symbols: Vec<u16>,
    pub ber_estimate: Option<f64>,
    pub snr_estimate: Option<f64>,
}}
```

### WaveformFactory Registration

After creating your waveform module, register it in `mod.rs`:

```rust
// 1. Add module: pub mod your_waveform;
// 2. Add to list(): vec![..., "YOUR-WAVEFORM"]
// 3. Add to create():
//    "YOURWAVEFORM" => Some(Box::new(your_waveform::YourWaveform::new(...)))
```

### Testing Requirements

```rust
#[test]
fn test_roundtrip() {{
    let waveform = YourWaveform::new(...);
    let original = vec![0, 1, 0, 1, 1, 0, 0, 1];
    let samples = waveform.modulate(&original);
    let result = waveform.demodulate(&samples);
    assert_eq!(result.bits, original);
}}
```

---

## Waveform Specification

{yaml}

---

## Implementation Request

Based on the R4W platform context and waveform specification above, please implement:

1. **The waveform module** (`crates/r4w-core/src/waveform/<name>.rs`)
2. **Module registration** in `mod.rs`
3. **Tests** for roundtrip and sample count

Focus on correct signal generation, clean Rust code, and educational value.
"#,
                yaml = yaml
            )
        } else {
            yaml
        }
    }

    pub fn render(&mut self, ui: &mut Ui) {
        ui.heading("Waveform Specification Wizard");
        ui.add_space(5.0);
        ui.label("Create a waveform specification following the R4W schema.");
        ui.add_space(10.0);

        // Step indicator
        ui.horizontal(|ui| {
            for step in [
                WizardStep::Identity,
                WizardStep::Modulation,
                WizardStep::SpreadSpectrum,
                WizardStep::PulseShaping,
                WizardStep::Timing,
                WizardStep::Coding,
                WizardStep::Spectral,
                WizardStep::Review,
            ] {
                let is_current = step == self.current_step;
                let color = if is_current {
                    Color32::from_rgb(100, 200, 100)
                } else {
                    Color32::GRAY
                };

                if ui.selectable_label(is_current, RichText::new(step.label()).color(color)).clicked() {
                    self.current_step = step;
                }
            }
        });

        ui.separator();

        // Step content
        egui::ScrollArea::vertical().show(ui, |ui| {
            match self.current_step {
                WizardStep::Identity => self.render_identity(ui),
                WizardStep::Modulation => self.render_modulation(ui),
                WizardStep::SpreadSpectrum => self.render_spread_spectrum(ui),
                WizardStep::PulseShaping => self.render_pulse_shaping(ui),
                WizardStep::Timing => self.render_timing(ui),
                WizardStep::Coding => self.render_coding(ui),
                WizardStep::Spectral => self.render_spectral(ui),
                WizardStep::Review => self.render_review(ui),
            }
        });

        ui.separator();

        // Navigation buttons
        ui.horizontal(|ui| {
            if let Some(prev) = self.current_step.prev() {
                if ui.button("<< Previous").clicked() {
                    self.current_step = prev;
                }
            }

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if let Some(next) = self.current_step.next() {
                    if ui.button("Next >>").clicked() {
                        self.current_step = next;
                    }
                } else {
                    if ui.button("Export").clicked() {
                        self.yaml_output = self.generate_export_content();
                        self.show_export_dialog = true;
                    }
                }
            });
        });

        // Export dialog
        if self.show_export_dialog {
            egui::Window::new("Export Waveform Specification")
                .resizable(true)
                .default_width(600.0)
                .default_height(400.0)
                .show(ui.ctx(), |ui| {
                    ui.horizontal(|ui| {
                        if ui.checkbox(&mut self.include_implementation_prompt, "Include R4W Implementation Prompt").changed() {
                            // Regenerate output with/without prompt
                            self.yaml_output = self.generate_export_content();
                        }
                    });
                    if self.include_implementation_prompt {
                        ui.label(RichText::new("Output includes full R4W context for AI-assisted implementation.").weak().italics());
                    } else {
                        ui.label(RichText::new("Spec-only mode. Combine with IMPLEMENTATION_PROMPT.md for AI implementation.").weak().italics());
                    }

                    ui.add_space(5.0);
                    ui.label("Copy the content below:");
                    egui::ScrollArea::vertical().max_height(280.0).show(ui, |ui| {
                        ui.add(egui::TextEdit::multiline(&mut self.yaml_output)
                            .font(egui::TextStyle::Monospace)
                            .desired_width(f32::INFINITY));
                    });

                    ui.horizontal(|ui| {
                        if ui.button("Copy to Clipboard").clicked() {
                            ui.output_mut(|o| o.copied_text = self.yaml_output.clone());
                        }
                        if ui.button("Save to File").clicked() {
                            let ext = if self.include_implementation_prompt { "md" } else { "yaml" };
                            if let Some(path) = rfd::FileDialog::new()
                                .set_file_name(&format!("{}.{}", self.spec.name, ext))
                                .add_filter("All", &["yaml", "yml", "md"])
                                .save_file()
                            {
                                if let Err(e) = std::fs::write(&path, &self.yaml_output) {
                                    log::error!("Failed to save file: {}", e);
                                }
                            }
                        }
                        if ui.button("Close").clicked() {
                            self.show_export_dialog = false;
                        }
                    });
                });
        }
    }

    fn render_identity(&mut self, ui: &mut Ui) {
        ui.heading("Waveform Identity");
        ui.add_space(10.0);

        // Preset selector
        ui.group(|ui| {
            ui.label(RichText::new("Start from a preset:").strong());
            ui.add_space(5.0);

            let presets = [
                WaveformPreset::Custom,
                WaveformPreset::AmBroadcast,
                WaveformPreset::FmBroadcast,
                WaveformPreset::Bpsk,
                WaveformPreset::Qpsk,
                WaveformPreset::LoRaSF7,
                WaveformPreset::LoRaSF12,
                WaveformPreset::DsssGold,
                WaveformPreset::Fhss,
                WaveformPreset::Gmsk,
                WaveformPreset::Stanag4285,
            ];

            let current_label = self.selected_preset.label();
            egui::ComboBox::from_label("")
                .selected_text(current_label)
                .width(300.0)
                .show_ui(ui, |ui| {
                    for preset in presets {
                        let response = ui.selectable_value(
                            &mut self.selected_preset,
                            preset,
                            preset.label()
                        );
                        if response.changed() {
                            self.load_preset(preset);
                        }
                    }
                });

            ui.label(RichText::new(self.selected_preset.description()).weak().italics());
        });

        ui.add_space(10.0);
        ui.separator();
        ui.add_space(10.0);

        ui.horizontal(|ui| {
            ui.label("Name (ID):");
            ui.text_edit_singleline(&mut self.spec.name);
        });

        ui.horizontal(|ui| {
            ui.label("Full Name:");
            ui.text_edit_singleline(&mut self.spec.full_name);
        });

        ui.horizontal(|ui| {
            ui.label("Version:");
            ui.text_edit_singleline(&mut self.spec.version);
        });

        ui.label("Description:");
        ui.text_edit_multiline(&mut self.spec.description);

        ui.add_space(10.0);
        ui.label("Classification:");

        ui.horizontal(|ui| {
            ui.radio_value(&mut self.spec.waveform_type, WaveformType::Digital, "Digital");
            ui.radio_value(&mut self.spec.waveform_type, WaveformType::Analog, "Analog");
            ui.radio_value(&mut self.spec.waveform_type, WaveformType::Hybrid, "Hybrid");
        });

        ui.add_space(10.0);
        ui.checkbox(&mut self.spec.lpd_lpi_enabled, "LPD/LPI Design");

        if self.spec.lpd_lpi_enabled {
            ui.horizontal(|ui| {
                ui.label("Target Processing Gain (dB):");
                ui.add(egui::Slider::new(&mut self.spec.target_processing_gain_db, 10.0..=30.0));
            });
        }
    }

    fn load_preset(&mut self, preset: WaveformPreset) {
        self.spec = match preset {
            WaveformPreset::Custom => WaveformSpec::default(),

            WaveformPreset::AmBroadcast => WaveformSpec {
                name: "AM-BROADCAST".to_string(),
                full_name: "AM Broadcast (DSB-FC)".to_string(),
                version: "1.0".to_string(),
                description: "Standard Amplitude Modulation as used in AM broadcast radio.\nDouble-Sideband Full-Carrier (DSB-FC) with envelope detection.".to_string(),
                waveform_type: WaveformType::Analog,
                lpd_lpi_enabled: false,
                target_processing_gain_db: 0.0,
                domain: ModulationDomain::Amplitude,
                digital_scheme: DigitalScheme::Ook,
                analog_scheme: AnalogScheme::Am,
                use_analog: true,
                carrier_freq_hz: 1_000_000.0,
                am_modulation_index: 0.8,
                fm_deviation_hz: 0.0,
                spread_technique: SpreadTechnique::None,
                dsss_enabled: false,
                chip_rate: 0,
                chips_per_symbol: 1,
                pn_type: PnSequenceType::Lfsr,
                pn_length: 0,
                fhss_enabled: false,
                num_hop_channels: 0,
                hop_rate: 0,
                css_sf: 7,
                css_bandwidth_hz: 125_000,
                pulse_shaping_enabled: false,
                filter_type: FilterType::Rectangular,
                rolloff: 0.0,
                span_symbols: 1,
                bt_product: 0.0,
                symbol_rate: 0,
                sample_rate: 48000,
                burst_mode: false,
                burst_duration_ms: 0.0,
                fec_enabled: false,
                fec_type: FecType::Hamming,
                code_rate: "1/1".to_string(),
                interleaving_enabled: false,
                scrambling_enabled: false,
                bandwidth_hz: 10000.0,
                target_psd_dbm_hz: -50.0,
            },

            WaveformPreset::FmBroadcast => WaveformSpec {
                name: "FM-BROADCAST".to_string(),
                full_name: "FM Broadcast (WBFM)".to_string(),
                version: "1.0".to_string(),
                description: "Wideband FM as used in FM broadcast radio.\n75 kHz deviation, stereo capable with 19 kHz pilot.".to_string(),
                waveform_type: WaveformType::Analog,
                lpd_lpi_enabled: false,
                target_processing_gain_db: 0.0,
                domain: ModulationDomain::Frequency,
                digital_scheme: DigitalScheme::Fsk2,
                analog_scheme: AnalogScheme::Wbfm,
                use_analog: true,
                carrier_freq_hz: 100_000_000.0,
                am_modulation_index: 0.0,
                fm_deviation_hz: 75000.0,
                spread_technique: SpreadTechnique::None,
                dsss_enabled: false,
                chip_rate: 0,
                chips_per_symbol: 1,
                pn_type: PnSequenceType::Lfsr,
                pn_length: 0,
                fhss_enabled: false,
                num_hop_channels: 0,
                hop_rate: 0,
                css_sf: 7,
                css_bandwidth_hz: 125_000,
                pulse_shaping_enabled: false,
                filter_type: FilterType::Rectangular,
                rolloff: 0.0,
                span_symbols: 1,
                bt_product: 0.0,
                symbol_rate: 0,
                sample_rate: 192000,
                burst_mode: false,
                burst_duration_ms: 0.0,
                fec_enabled: false,
                fec_type: FecType::Hamming,
                code_rate: "1/1".to_string(),
                interleaving_enabled: false,
                scrambling_enabled: false,
                bandwidth_hz: 200000.0,
                target_psd_dbm_hz: -60.0,
            },

            WaveformPreset::Bpsk => WaveformSpec {
                name: "BPSK".to_string(),
                full_name: "Binary Phase Shift Keying".to_string(),
                version: "1.0".to_string(),
                description: "Simple and robust binary PSK modulation.\n1 bit per symbol, excellent BER performance.".to_string(),
                waveform_type: WaveformType::Digital,
                lpd_lpi_enabled: false,
                target_processing_gain_db: 0.0,
                domain: ModulationDomain::Phase,
                digital_scheme: DigitalScheme::Bpsk,
                analog_scheme: AnalogScheme::Am,
                use_analog: false,
                carrier_freq_hz: 1_000_000.0,
                am_modulation_index: 0.0,
                fm_deviation_hz: 0.0,
                spread_technique: SpreadTechnique::None,
                dsss_enabled: false,
                chip_rate: 0,
                chips_per_symbol: 1,
                pn_type: PnSequenceType::Gold,
                pn_length: 127,
                fhss_enabled: false,
                num_hop_channels: 0,
                hop_rate: 0,
                css_sf: 7,
                css_bandwidth_hz: 125_000,
                pulse_shaping_enabled: true,
                filter_type: FilterType::RootRaisedCosine,
                rolloff: 0.35,
                span_symbols: 8,
                bt_product: 0.3,
                symbol_rate: 9600,
                sample_rate: 48000,
                burst_mode: false,
                burst_duration_ms: 0.0,
                fec_enabled: true,
                fec_type: FecType::Convolutional,
                code_rate: "1/2".to_string(),
                interleaving_enabled: true,
                scrambling_enabled: true,
                bandwidth_hz: 12960.0,
                target_psd_dbm_hz: -100.0,
            },

            WaveformPreset::Qpsk => WaveformSpec {
                name: "QPSK".to_string(),
                full_name: "Quaternary Phase Shift Keying".to_string(),
                version: "1.0".to_string(),
                description: "Standard QPSK modulation with 2 bits per symbol.\nCommon in satellite and digital communications.".to_string(),
                waveform_type: WaveformType::Digital,
                lpd_lpi_enabled: false,
                target_processing_gain_db: 0.0,
                domain: ModulationDomain::Phase,
                digital_scheme: DigitalScheme::Qpsk,
                analog_scheme: AnalogScheme::Am,
                use_analog: false,
                carrier_freq_hz: 1_000_000.0,
                am_modulation_index: 0.0,
                fm_deviation_hz: 0.0,
                spread_technique: SpreadTechnique::None,
                dsss_enabled: false,
                chip_rate: 0,
                chips_per_symbol: 1,
                pn_type: PnSequenceType::Gold,
                pn_length: 127,
                fhss_enabled: false,
                num_hop_channels: 0,
                hop_rate: 0,
                css_sf: 7,
                css_bandwidth_hz: 125_000,
                pulse_shaping_enabled: true,
                filter_type: FilterType::RootRaisedCosine,
                rolloff: 0.35,
                span_symbols: 8,
                bt_product: 0.3,
                symbol_rate: 9600,
                sample_rate: 48000,
                burst_mode: false,
                burst_duration_ms: 0.0,
                fec_enabled: true,
                fec_type: FecType::Convolutional,
                code_rate: "1/2".to_string(),
                interleaving_enabled: true,
                scrambling_enabled: true,
                bandwidth_hz: 12960.0,
                target_psd_dbm_hz: -100.0,
            },

            WaveformPreset::LoRaSF7 => WaveformSpec {
                name: "LORA-SF7".to_string(),
                full_name: "LoRa Chirp Spread Spectrum SF7".to_string(),
                version: "1.0".to_string(),
                description: "LoRa modulation with SF7 for high data rate.\nChirp spread spectrum with 128 chips per symbol.".to_string(),
                waveform_type: WaveformType::Digital,
                lpd_lpi_enabled: true,
                target_processing_gain_db: 21.0,
                domain: ModulationDomain::Frequency,
                digital_scheme: DigitalScheme::Css,
                analog_scheme: AnalogScheme::Am,
                use_analog: false,
                carrier_freq_hz: 915_000_000.0,
                am_modulation_index: 0.0,
                fm_deviation_hz: 0.0,
                spread_technique: SpreadTechnique::Css,
                dsss_enabled: false,
                chip_rate: 125_000,
                chips_per_symbol: 128,
                pn_type: PnSequenceType::Lfsr,
                pn_length: 128,
                fhss_enabled: false,
                num_hop_channels: 0,
                hop_rate: 0,
                css_sf: 7,
                css_bandwidth_hz: 125_000,
                pulse_shaping_enabled: false,
                filter_type: FilterType::Rectangular,
                rolloff: 0.0,
                span_symbols: 1,
                bt_product: 0.0,
                symbol_rate: 976,
                sample_rate: 125_000,
                burst_mode: false,
                burst_duration_ms: 0.0,
                fec_enabled: true,
                fec_type: FecType::Hamming,
                code_rate: "4/5".to_string(),
                interleaving_enabled: true,
                scrambling_enabled: true,
                bandwidth_hz: 125_000.0,
                target_psd_dbm_hz: -130.0,
            },

            WaveformPreset::LoRaSF12 => WaveformSpec {
                name: "LORA-SF12".to_string(),
                full_name: "LoRa Chirp Spread Spectrum SF12".to_string(),
                version: "1.0".to_string(),
                description: "LoRa modulation with SF12 for maximum range.\n4096 chips per symbol, -20 dB below noise floor.".to_string(),
                waveform_type: WaveformType::Digital,
                lpd_lpi_enabled: true,
                target_processing_gain_db: 36.0,
                domain: ModulationDomain::Frequency,
                digital_scheme: DigitalScheme::Css,
                analog_scheme: AnalogScheme::Am,
                use_analog: false,
                carrier_freq_hz: 915_000_000.0,
                am_modulation_index: 0.0,
                fm_deviation_hz: 0.0,
                spread_technique: SpreadTechnique::Css,
                dsss_enabled: false,
                chip_rate: 125_000,
                chips_per_symbol: 4096,
                pn_type: PnSequenceType::Lfsr,
                pn_length: 4096,
                fhss_enabled: false,
                num_hop_channels: 0,
                hop_rate: 0,
                css_sf: 12,
                css_bandwidth_hz: 125_000,
                pulse_shaping_enabled: false,
                filter_type: FilterType::Rectangular,
                rolloff: 0.0,
                span_symbols: 1,
                bt_product: 0.0,
                symbol_rate: 30,
                sample_rate: 125_000,
                burst_mode: false,
                burst_duration_ms: 0.0,
                fec_enabled: true,
                fec_type: FecType::Hamming,
                code_rate: "4/5".to_string(),
                interleaving_enabled: true,
                scrambling_enabled: true,
                bandwidth_hz: 125_000.0,
                target_psd_dbm_hz: -140.0,
            },

            WaveformPreset::DsssGold => WaveformSpec {
                name: "DSSS-GOLD".to_string(),
                full_name: "DSSS with Gold Codes".to_string(),
                version: "1.0".to_string(),
                description: "Direct Sequence Spread Spectrum with Gold codes.\nLPD/LPI capable with 21 dB processing gain.".to_string(),
                waveform_type: WaveformType::Digital,
                lpd_lpi_enabled: true,
                target_processing_gain_db: 21.0,
                domain: ModulationDomain::Phase,
                digital_scheme: DigitalScheme::Bpsk,
                analog_scheme: AnalogScheme::Am,
                use_analog: false,
                carrier_freq_hz: 2_400_000_000.0,
                am_modulation_index: 0.0,
                fm_deviation_hz: 0.0,
                spread_technique: SpreadTechnique::Dsss,
                dsss_enabled: true,
                chip_rate: 1_000_000,
                chips_per_symbol: 127,
                pn_type: PnSequenceType::Gold,
                pn_length: 127,
                fhss_enabled: false,
                num_hop_channels: 0,
                hop_rate: 0,
                css_sf: 7,
                css_bandwidth_hz: 125_000,
                pulse_shaping_enabled: true,
                filter_type: FilterType::RootRaisedCosine,
                rolloff: 0.35,
                span_symbols: 8,
                bt_product: 0.3,
                symbol_rate: 7874,
                sample_rate: 2_000_000,
                burst_mode: false,
                burst_duration_ms: 0.0,
                fec_enabled: true,
                fec_type: FecType::Convolutional,
                code_rate: "1/2".to_string(),
                interleaving_enabled: true,
                scrambling_enabled: true,
                bandwidth_hz: 2_000_000.0,
                target_psd_dbm_hz: -130.0,
            },

            WaveformPreset::Fhss => WaveformSpec {
                name: "FHSS-TACTICAL".to_string(),
                full_name: "Frequency Hopping Spread Spectrum".to_string(),
                version: "1.0".to_string(),
                description: "Military-style frequency hopping.\n50 channels, 100 hops/sec, jam resistant.".to_string(),
                waveform_type: WaveformType::Digital,
                lpd_lpi_enabled: true,
                target_processing_gain_db: 17.0,
                domain: ModulationDomain::Phase,
                digital_scheme: DigitalScheme::Qpsk,
                analog_scheme: AnalogScheme::Am,
                use_analog: false,
                carrier_freq_hz: 300_000_000.0,
                am_modulation_index: 0.0,
                fm_deviation_hz: 0.0,
                spread_technique: SpreadTechnique::Fhss,
                dsss_enabled: false,
                chip_rate: 0,
                chips_per_symbol: 1,
                pn_type: PnSequenceType::Gold,
                pn_length: 127,
                fhss_enabled: true,
                num_hop_channels: 50,
                hop_rate: 100,
                css_sf: 7,
                css_bandwidth_hz: 125_000,
                pulse_shaping_enabled: true,
                filter_type: FilterType::RootRaisedCosine,
                rolloff: 0.35,
                span_symbols: 8,
                bt_product: 0.3,
                symbol_rate: 4800,
                sample_rate: 48000,
                burst_mode: true,
                burst_duration_ms: 10.0,
                fec_enabled: true,
                fec_type: FecType::Convolutional,
                code_rate: "1/2".to_string(),
                interleaving_enabled: true,
                scrambling_enabled: true,
                bandwidth_hz: 25000.0,
                target_psd_dbm_hz: -120.0,
            },

            WaveformPreset::Gmsk => WaveformSpec {
                name: "GMSK".to_string(),
                full_name: "Gaussian Minimum Shift Keying".to_string(),
                version: "1.0".to_string(),
                description: "Constant envelope modulation as used in GSM.\nBT=0.3, spectrally efficient, good for nonlinear amplifiers.".to_string(),
                waveform_type: WaveformType::Digital,
                lpd_lpi_enabled: false,
                target_processing_gain_db: 0.0,
                domain: ModulationDomain::Frequency,
                digital_scheme: DigitalScheme::Gmsk,
                analog_scheme: AnalogScheme::Am,
                use_analog: false,
                carrier_freq_hz: 900_000_000.0,
                am_modulation_index: 0.0,
                fm_deviation_hz: 0.0,
                spread_technique: SpreadTechnique::None,
                dsss_enabled: false,
                chip_rate: 0,
                chips_per_symbol: 1,
                pn_type: PnSequenceType::Lfsr,
                pn_length: 0,
                fhss_enabled: false,
                num_hop_channels: 0,
                hop_rate: 0,
                css_sf: 7,
                css_bandwidth_hz: 125_000,
                pulse_shaping_enabled: true,
                filter_type: FilterType::Gaussian,
                rolloff: 0.0,
                span_symbols: 4,
                bt_product: 0.3,
                symbol_rate: 270833,
                sample_rate: 1_000_000,
                burst_mode: true,
                burst_duration_ms: 0.577,
                fec_enabled: true,
                fec_type: FecType::Convolutional,
                code_rate: "1/2".to_string(),
                interleaving_enabled: true,
                scrambling_enabled: true,
                bandwidth_hz: 200_000.0,
                target_psd_dbm_hz: -90.0,
            },

            WaveformPreset::Stanag4285 => WaveformSpec {
                name: "STANAG-4285".to_string(),
                full_name: "STANAG 4285 HF Data Modem".to_string(),
                version: "1.0".to_string(),
                description: "NATO standard HF modem for robust data transfer.\n8-PSK with convolutional coding, 2400 baud.".to_string(),
                waveform_type: WaveformType::Digital,
                lpd_lpi_enabled: false,
                target_processing_gain_db: 0.0,
                domain: ModulationDomain::Phase,
                digital_scheme: DigitalScheme::Qpsk, // Actually 8-PSK but using QPSK as closest
                analog_scheme: AnalogScheme::Am,
                use_analog: false,
                carrier_freq_hz: 1_800_000.0,
                am_modulation_index: 0.0,
                fm_deviation_hz: 0.0,
                spread_technique: SpreadTechnique::None,
                dsss_enabled: false,
                chip_rate: 0,
                chips_per_symbol: 1,
                pn_type: PnSequenceType::Lfsr,
                pn_length: 0,
                fhss_enabled: false,
                num_hop_channels: 0,
                hop_rate: 0,
                css_sf: 7,
                css_bandwidth_hz: 125_000,
                pulse_shaping_enabled: true,
                filter_type: FilterType::RootRaisedCosine,
                rolloff: 0.25,
                span_symbols: 8,
                bt_product: 0.3,
                symbol_rate: 2400,
                sample_rate: 48000,
                burst_mode: false,
                burst_duration_ms: 0.0,
                fec_enabled: true,
                fec_type: FecType::Convolutional,
                code_rate: "1/2".to_string(),
                interleaving_enabled: true,
                scrambling_enabled: true,
                bandwidth_hz: 3000.0,
                target_psd_dbm_hz: -80.0,
            },
        };
    }

    fn render_modulation(&mut self, ui: &mut Ui) {
        ui.heading("Modulation");
        ui.add_space(10.0);

        ui.label("Primary Domain:");
        ui.horizontal(|ui| {
            ui.radio_value(&mut self.spec.domain, ModulationDomain::Phase, "Phase (PSK)");
            ui.radio_value(&mut self.spec.domain, ModulationDomain::Frequency, "Frequency (FSK)");
            ui.radio_value(&mut self.spec.domain, ModulationDomain::Amplitude, "Amplitude (ASK)");
        });

        ui.add_space(10.0);
        ui.checkbox(&mut self.spec.use_analog, "Analog Modulation");

        if self.spec.use_analog {
            ui.horizontal(|ui| {
                ui.radio_value(&mut self.spec.analog_scheme, AnalogScheme::Am, "AM");
                ui.radio_value(&mut self.spec.analog_scheme, AnalogScheme::Nbfm, "NBFM");
                ui.radio_value(&mut self.spec.analog_scheme, AnalogScheme::Wbfm, "WBFM");
                ui.radio_value(&mut self.spec.analog_scheme, AnalogScheme::SsbUsb, "SSB-USB");
            });

            ui.horizontal(|ui| {
                ui.label("Carrier Frequency (Hz):");
                ui.add(egui::DragValue::new(&mut self.spec.carrier_freq_hz).speed(1000.0));
            });

            if matches!(self.spec.analog_scheme, AnalogScheme::Am | AnalogScheme::DsbSc) {
                ui.horizontal(|ui| {
                    ui.label("Modulation Index:");
                    ui.add(egui::Slider::new(&mut self.spec.am_modulation_index, 0.0..=1.0));
                });
            }

            if matches!(self.spec.analog_scheme, AnalogScheme::Nbfm | AnalogScheme::Wbfm) {
                ui.horizontal(|ui| {
                    ui.label("Frequency Deviation (Hz):");
                    ui.add(egui::DragValue::new(&mut self.spec.fm_deviation_hz).speed(100.0));
                });
            }
        } else {
            ui.add_space(10.0);
            ui.label("Digital Scheme:");
            egui::ComboBox::from_label("")
                .selected_text(format!("{:?}", self.spec.digital_scheme))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Bpsk, "BPSK");
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Qpsk, "QPSK");
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Pi4Qpsk, "pi/4-QPSK");
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Qam16, "16-QAM");
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Qam64, "64-QAM");
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Fsk2, "2-FSK");
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Fsk4, "4-FSK");
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Gmsk, "GMSK");
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Msk, "MSK");
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Ook, "OOK");
                    ui.selectable_value(&mut self.spec.digital_scheme, DigitalScheme::Css, "CSS (LoRa)");
                });

            ui.add_space(5.0);
            let order = self.spec.digital_scheme.order();
            let bits = (order as f32).log2() as u32;
            ui.label(format!("Order: {} symbols ({} bits/symbol)", order, bits));
        }
    }

    fn render_spread_spectrum(&mut self, ui: &mut Ui) {
        ui.heading("Spread Spectrum");
        ui.add_space(10.0);

        ui.label("Technique:");
        ui.horizontal(|ui| {
            ui.radio_value(&mut self.spec.spread_technique, SpreadTechnique::None, "None");
            ui.radio_value(&mut self.spec.spread_technique, SpreadTechnique::Dsss, "DSSS");
            ui.radio_value(&mut self.spec.spread_technique, SpreadTechnique::Fhss, "FHSS");
            ui.radio_value(&mut self.spec.spread_technique, SpreadTechnique::Css, "CSS");
        });

        ui.add_space(10.0);

        match self.spec.spread_technique {
            SpreadTechnique::Dsss => {
                self.spec.dsss_enabled = true;
                self.spec.fhss_enabled = false;

                ui.group(|ui| {
                    ui.label("DSSS Configuration");

                    ui.horizontal(|ui| {
                        ui.label("Chip Rate (chips/s):");
                        ui.add(egui::DragValue::new(&mut self.spec.chip_rate).speed(10000));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Chips per Symbol:");
                        ui.add(egui::DragValue::new(&mut self.spec.chips_per_symbol).speed(1).range(7..=1023));
                    });

                    ui.label("PN Sequence Type:");
                    egui::ComboBox::from_id_salt("pn_type")
                        .selected_text(format!("{:?}", self.spec.pn_type))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.spec.pn_type, PnSequenceType::Gold, "Gold");
                            ui.selectable_value(&mut self.spec.pn_type, PnSequenceType::Kasami, "Kasami");
                            ui.selectable_value(&mut self.spec.pn_type, PnSequenceType::Barker, "Barker");
                            ui.selectable_value(&mut self.spec.pn_type, PnSequenceType::Lfsr, "LFSR");
                        });

                    ui.horizontal(|ui| {
                        ui.label("Sequence Length:");
                        ui.add(egui::DragValue::new(&mut self.spec.pn_length).speed(1));
                    });

                    let pg = 10.0 * (self.spec.chips_per_symbol as f32).log10();
                    ui.label(format!("Processing Gain: {:.1} dB", pg));
                });
            }
            SpreadTechnique::Fhss => {
                self.spec.dsss_enabled = false;
                self.spec.fhss_enabled = true;

                ui.group(|ui| {
                    ui.label("FHSS Configuration");

                    ui.horizontal(|ui| {
                        ui.label("Number of Channels:");
                        ui.add(egui::DragValue::new(&mut self.spec.num_hop_channels).speed(1).range(2..=1000));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Hop Rate (hops/s):");
                        ui.add(egui::DragValue::new(&mut self.spec.hop_rate).speed(1).range(1..=10000));
                    });

                    let dwell_time = 1000.0 / self.spec.hop_rate as f32;
                    ui.label(format!("Dwell Time: {:.1} ms", dwell_time));
                });
            }
            SpreadTechnique::Css => {
                self.spec.dsss_enabled = false;
                self.spec.fhss_enabled = false;

                ui.group(|ui| {
                    ui.label("CSS (Chirp Spread Spectrum) Configuration");

                    ui.horizontal(|ui| {
                        ui.label("Spreading Factor:");
                        ui.add(egui::Slider::new(&mut self.spec.css_sf, 5..=12));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Bandwidth (Hz):");
                        egui::ComboBox::from_id_salt("css_bw")
                            .selected_text(format!("{} kHz", self.spec.css_bandwidth_hz / 1000))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut self.spec.css_bandwidth_hz, 125_000, "125 kHz");
                                ui.selectable_value(&mut self.spec.css_bandwidth_hz, 250_000, "250 kHz");
                                ui.selectable_value(&mut self.spec.css_bandwidth_hz, 500_000, "500 kHz");
                            });
                    });

                    let symbols = 1 << self.spec.css_sf;
                    let symbol_time = symbols as f32 / self.spec.css_bandwidth_hz as f32 * 1000.0;
                    ui.label(format!("Symbols: {}, Symbol Time: {:.2} ms", symbols, symbol_time));
                });
            }
            _ => {
                self.spec.dsss_enabled = false;
                self.spec.fhss_enabled = false;
                ui.label("No spread spectrum selected.");
            }
        }
    }

    fn render_pulse_shaping(&mut self, ui: &mut Ui) {
        ui.heading("Pulse Shaping");
        ui.add_space(10.0);

        ui.checkbox(&mut self.spec.pulse_shaping_enabled, "Enable Pulse Shaping");

        if self.spec.pulse_shaping_enabled {
            ui.add_space(10.0);

            ui.label("Filter Type:");
            ui.horizontal(|ui| {
                ui.radio_value(&mut self.spec.filter_type, FilterType::RootRaisedCosine, "Root Raised Cosine");
                ui.radio_value(&mut self.spec.filter_type, FilterType::RaisedCosine, "Raised Cosine");
                ui.radio_value(&mut self.spec.filter_type, FilterType::Gaussian, "Gaussian");
                ui.radio_value(&mut self.spec.filter_type, FilterType::Rectangular, "Rectangular");
            });

            match self.spec.filter_type {
                FilterType::RaisedCosine | FilterType::RootRaisedCosine => {
                    ui.horizontal(|ui| {
                        ui.label("Roll-off Factor:");
                        ui.add(egui::Slider::new(&mut self.spec.rolloff, 0.0..=1.0));
                    });

                    ui.horizontal(|ui| {
                        ui.label("Span (symbols):");
                        ui.add(egui::Slider::new(&mut self.spec.span_symbols, 4..=16));
                    });
                }
                FilterType::Gaussian => {
                    ui.horizontal(|ui| {
                        ui.label("BT Product:");
                        ui.add(egui::Slider::new(&mut self.spec.bt_product, 0.2..=0.5));
                    });
                }
                _ => {}
            }
        }
    }

    fn render_timing(&mut self, ui: &mut Ui) {
        ui.heading("Timing");
        ui.add_space(10.0);

        ui.horizontal(|ui| {
            ui.label("Symbol Rate (symbols/s):");
            ui.add(egui::DragValue::new(&mut self.spec.symbol_rate).speed(10).range(1..=1_000_000));
        });

        ui.horizontal(|ui| {
            ui.label("Sample Rate (Hz):");
            ui.add(egui::DragValue::new(&mut self.spec.sample_rate).speed(100).range(1000..=10_000_000));
        });

        let sps = self.spec.sample_rate as f32 / self.spec.symbol_rate as f32;
        ui.label(format!("Samples per Symbol: {:.1}", sps));

        ui.add_space(10.0);
        ui.checkbox(&mut self.spec.burst_mode, "Burst Mode (for LPD)");

        if self.spec.burst_mode {
            ui.horizontal(|ui| {
                ui.label("Burst Duration (ms):");
                ui.add(egui::Slider::new(&mut self.spec.burst_duration_ms, 1.0..=100.0));
            });
        }
    }

    fn render_coding(&mut self, ui: &mut Ui) {
        ui.heading("Channel Coding");
        ui.add_space(10.0);

        ui.checkbox(&mut self.spec.fec_enabled, "Enable Forward Error Correction");

        if self.spec.fec_enabled {
            ui.add_space(5.0);

            ui.label("FEC Type:");
            ui.horizontal(|ui| {
                ui.radio_value(&mut self.spec.fec_type, FecType::Convolutional, "Convolutional");
                ui.radio_value(&mut self.spec.fec_type, FecType::Ldpc, "LDPC");
                ui.radio_value(&mut self.spec.fec_type, FecType::Turbo, "Turbo");
                ui.radio_value(&mut self.spec.fec_type, FecType::Hamming, "Hamming");
            });

            ui.horizontal(|ui| {
                ui.label("Code Rate:");
                egui::ComboBox::from_id_salt("code_rate")
                    .selected_text(&self.spec.code_rate)
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.spec.code_rate, "1/2".to_string(), "1/2");
                        ui.selectable_value(&mut self.spec.code_rate, "2/3".to_string(), "2/3");
                        ui.selectable_value(&mut self.spec.code_rate, "3/4".to_string(), "3/4");
                        ui.selectable_value(&mut self.spec.code_rate, "4/5".to_string(), "4/5");
                    });
            });
        }

        ui.add_space(10.0);
        ui.checkbox(&mut self.spec.interleaving_enabled, "Enable Interleaving");
        ui.checkbox(&mut self.spec.scrambling_enabled, "Enable Scrambling");
    }

    fn render_spectral(&mut self, ui: &mut Ui) {
        ui.heading("Spectral Characteristics");
        ui.add_space(10.0);

        ui.horizontal(|ui| {
            ui.label("Occupied Bandwidth (Hz):");
            ui.add(egui::DragValue::new(&mut self.spec.bandwidth_hz).speed(100.0));
        });

        if self.spec.lpd_lpi_enabled {
            ui.horizontal(|ui| {
                ui.label("Target PSD (dBm/Hz):");
                ui.add(egui::Slider::new(&mut self.spec.target_psd_dbm_hz, -150.0..=-100.0));
            });

            let thermal_noise = -174.0_f32; // dBm/Hz at room temp
            let margin = thermal_noise - self.spec.target_psd_dbm_hz;
            let color = if margin > 0.0 { Color32::GREEN } else { Color32::RED };
            ui.colored_label(color, format!("Margin below thermal noise: {:.1} dB", margin));
        }
    }

    fn render_review(&mut self, ui: &mut Ui) {
        ui.heading("Review Configuration");
        ui.add_space(10.0);

        ui.group(|ui| {
            ui.label(RichText::new("Summary").strong());

            ui.label(format!("Waveform: {} ({})", self.spec.name, self.spec.full_name));
            ui.label(format!("Type: {:?}", self.spec.waveform_type));

            if self.spec.use_analog {
                ui.label(format!("Modulation: {:?}", self.spec.analog_scheme));
            } else {
                ui.label(format!("Modulation: {:?} ({:?})", self.spec.digital_scheme, self.spec.domain));
            }

            ui.label(format!("Spread Spectrum: {:?}", self.spec.spread_technique));

            if self.spec.dsss_enabled {
                ui.label(format!("  - Chips/Symbol: {}", self.spec.chips_per_symbol));
                ui.label(format!("  - Processing Gain: {:.1} dB", self.spec.compute_processing_gain()));
            }

            ui.label(format!("Symbol Rate: {} sym/s", self.spec.symbol_rate));
            ui.label(format!("Data Rate: {} bps", self.spec.compute_data_rate()));
            ui.label(format!("Spectral Efficiency: {:.4} bps/Hz", self.spec.compute_spectral_efficiency()));

            if self.spec.fec_enabled {
                ui.label(format!("FEC: {:?} (rate {})", self.spec.fec_type, self.spec.code_rate));
            }

            if self.spec.lpd_lpi_enabled {
                ui.label(format!("LPD/LPI: Target {} dB processing gain", self.spec.target_processing_gain_db));
            }
        });

        ui.add_space(10.0);
        if ui.button("Generate YAML Preview").clicked() {
            self.yaml_output = self.spec.to_yaml();
        }

        if !self.yaml_output.is_empty() {
            ui.add_space(10.0);
            egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
                ui.add(egui::TextEdit::multiline(&mut self.yaml_output.as_str())
                    .font(egui::TextStyle::Monospace)
                    .desired_width(f32::INFINITY));
            });
        }
    }
}
