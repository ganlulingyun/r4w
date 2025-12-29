//! Overview view - Introduction to SDR and waveform-specific concepts

use egui::Ui;
use r4w_core::params::LoRaParams;
use r4w_core::types::IQSample;

/// Tab selection for the Overview view
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OverviewTab {
    /// Selected waveform-specific content
    #[default]
    Waveform,
    /// SDR basics and background
    SdrBasics,
    /// R4W platform overview
    Platform,
}

pub struct OverviewView {
    /// Currently selected tab
    selected_tab: OverviewTab,
}

impl Default for OverviewView {
    fn default() -> Self {
        Self::new()
    }
}

impl OverviewView {
    pub fn new() -> Self {
        Self {
            selected_tab: OverviewTab::Waveform,
        }
    }

    pub fn render(
        &mut self,
        ui: &mut Ui,
        waveform_name: &str,
        params: &LoRaParams,
        _samples: &Option<Vec<IQSample>>,
    ) {
        // Horizontal tabs at the top
        ui.horizontal(|ui| {
            ui.selectable_value(
                &mut self.selected_tab,
                OverviewTab::Waveform,
                format!("{} Overview", waveform_name),
            );
            ui.selectable_value(
                &mut self.selected_tab,
                OverviewTab::SdrBasics,
                "SDR Basics",
            );
            ui.selectable_value(
                &mut self.selected_tab,
                OverviewTab::Platform,
                "R4W Platform",
            );
        });

        ui.separator();
        ui.add_space(8.0);

        // Content based on selected tab
        egui::ScrollArea::vertical().show(ui, |ui| {
            match self.selected_tab {
                OverviewTab::Waveform => {
                    self.render_waveform_content(ui, waveform_name, params);
                }
                OverviewTab::SdrBasics => {
                    self.render_sdr_basics(ui);
                }
                OverviewTab::Platform => {
                    self.render_platform_overview(ui);
                }
            }
        });
    }

    /// Render comprehensive SDR basics content
    fn render_sdr_basics(&self, ui: &mut Ui) {
        // What is SDR
        ui.heading("What is Software Defined Radio?");
        ui.add_space(8.0);

        ui.label(
            "Software Defined Radio (SDR) is a radio communication system where components \
            traditionally implemented in hardware (mixers, filters, modulators/demodulators) \
            are instead implemented in software.",
        );
        ui.add_space(12.0);

        ui.label("Key advantages of SDR:");
        ui.indent("sdr_advantages", |ui| {
            ui.label("• Flexibility: Change waveforms with software updates");
            ui.label("• Cost: Single hardware platform, multiple protocols");
            ui.label("• Rapid prototyping: Test new modulation schemes quickly");
            ui.label("• Education: See inside the radio processing pipeline");
        });

        ui.add_space(16.0);
        ui.separator();

        // I/Q Explanation
        ui.heading("Understanding I/Q Samples");
        ui.add_space(8.0);

        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 0.0;
            ui.label("SDR works with ");
            ui.label(egui::RichText::new("complex numbers").strong());
            ui.label(" called I/Q samples, where:");
        });

        ui.add_space(4.0);
        ui.indent("iq_indent", |ui| {
            ui.label("• I (In-phase): The real component");
            ui.label("• Q (Quadrature): The imaginary component (90° phase shifted)");
        });
        ui.add_space(8.0);

        ui.label(
            "Together, I and Q capture both the amplitude AND phase of a signal, \
            which is essential for digital modulation schemes.",
        );

        ui.add_space(12.0);
        ui.code(
            r#"
       Q (Imaginary)
           ^
           |     * (I=0.7, Q=0.7)
           |    /   Amplitude = 1.0
           |   /    Phase = 45°
           |  /
           | /
   --------+--------> I (Real)
           |
"#,
        );

        ui.add_space(16.0);
        ui.separator();

        // Modulation types
        ui.heading("Types of Modulation");
        ui.add_space(8.0);

        ui.label("Digital modulation encodes data by varying signal properties:");
        ui.add_space(8.0);

        egui::Grid::new("mod_types_grid")
            .num_columns(2)
            .spacing([40.0, 8.0])
            .show(ui, |ui| {
                ui.label(egui::RichText::new("Amplitude").strong());
                ui.label("OOK, ASK, QAM - vary the signal strength");
                ui.end_row();

                ui.label(egui::RichText::new("Frequency").strong());
                ui.label("FSK, FHSS - vary the carrier frequency");
                ui.end_row();

                ui.label(egui::RichText::new("Phase").strong());
                ui.label("PSK, QPSK, 8PSK - vary the carrier phase");
                ui.end_row();

                ui.label(egui::RichText::new("Combined").strong());
                ui.label("QAM - vary both amplitude and phase");
                ui.end_row();

                ui.label(egui::RichText::new("Spread Spectrum").strong());
                ui.label("DSSS, FHSS, CSS - spread energy across bandwidth");
                ui.end_row();
            });

        ui.add_space(16.0);
        ui.separator();

        // Spectral efficiency
        ui.heading("Spectral Efficiency: Bits per Symbol");
        ui.add_space(8.0);

        ui.label(
            "Different modulation schemes encode different amounts of data per symbol. \
            Higher-order modulation packs more bits but requires better SNR:",
        );

        ui.add_space(8.0);
        ui.code(
            r#"
Modulation    Bits/Symbol    Min SNR (BER=10⁻⁵)
───────────────────────────────────────────────
BPSK               1              ~9.6 dB
QPSK               2              ~9.6 dB
8-PSK              3              ~13 dB
16-QAM             4              ~17 dB
64-QAM             6              ~23 dB
256-QAM            8              ~29 dB
───────────────────────────────────────────────
"#,
        );

        ui.add_space(16.0);
        ui.separator();

        // Signal processing pipeline
        ui.heading("Typical SDR Processing Pipeline");
        ui.add_space(8.0);

        ui.code(
            r#"
TRANSMIT PATH:
  Data Bits ──► FEC Encode ──► Interleave ──► Symbol Map ──► Pulse Shape ──► Upconvert ──► I/Q Out
                   │              │               │              │
               Add error      Spread         Bits to       Filter for
               correction     errors         constellation  bandwidth

RECEIVE PATH:
  I/Q In ──► Downconvert ──► Filter ──► Sync ──► Symbol Detect ──► De-interleave ──► FEC Decode ──► Data
                │             │         │              │
             Remove       Matched    Carrier/      Find nearest
             carrier      filter     timing sync   constellation pt
"#,
        );

        ui.add_space(16.0);
        ui.separator();

        // Key concepts
        ui.heading("Key SDR Concepts");
        ui.add_space(8.0);

        egui::CollapsingHeader::new("Sample Rate and Bandwidth")
            .default_open(false)
            .show(ui, |ui| {
                ui.label(
                    "The sample rate determines the maximum signal bandwidth you can capture \
                    (Nyquist: BW = Fs/2). Higher sample rates allow wider bandwidth signals \
                    but require more processing power.",
                );
            });

        egui::CollapsingHeader::new("SNR and BER")
            .default_open(false)
            .show(ui, |ui| {
                ui.label(
                    "Signal-to-Noise Ratio (SNR) measures signal quality in dB. \
                    Bit Error Rate (BER) measures the fraction of bits received incorrectly. \
                    Higher SNR → Lower BER → More reliable communication.",
                );
            });

        egui::CollapsingHeader::new("Synchronization")
            .default_open(false)
            .show(ui, |ui| {
                ui.label(
                    "Receivers must synchronize with transmitters in multiple domains:\n\
                    • Frame sync: Where does the packet start?\n\
                    • Symbol timing: When to sample each symbol?\n\
                    • Carrier frequency: Compensate for oscillator offset (CFO)\n\
                    • Phase: Track phase variations over time",
                );
            });

        egui::CollapsingHeader::new("Channel Effects")
            .default_open(false)
            .show(ui, |ui| {
                ui.label(
                    "Real channels introduce:\n\
                    • AWGN: Additive White Gaussian Noise (thermal noise)\n\
                    • Multipath: Signal arrives via multiple paths (fading)\n\
                    • Doppler: Frequency shift from relative motion\n\
                    • Interference: Other signals in the same band",
                );
            });
    }

    /// Render R4W platform overview
    fn render_platform_overview(&self, ui: &mut Ui) {
        ui.heading("R4W - Rust for Waveforms");
        ui.add_space(8.0);

        ui.label(
            "R4W is a platform for developing, testing, and deploying SDR waveforms in Rust. \
            It provides reusable DSP libraries, educational tools, and production-ready components.",
        );

        ui.add_space(12.0);
        ui.heading("Why Rust for SDR?");
        ui.add_space(8.0);

        egui::Grid::new("rust_benefits_grid")
            .num_columns(2)
            .spacing([40.0, 4.0])
            .show(ui, |ui| {
                ui.label(egui::RichText::new("Memory Safety").strong());
                ui.label("No buffer overflows in signal processing");
                ui.end_row();

                ui.label(egui::RichText::new("Zero-Cost Abstractions").strong());
                ui.label("High-level APIs with C-level performance");
                ui.end_row();

                ui.label(egui::RichText::new("Fearless Concurrency").strong());
                ui.label("Safe parallel DSP processing");
                ui.end_row();

                ui.label(egui::RichText::new("Cross-Compilation").strong());
                ui.label("ARM, x86, embedded, and WASM from one codebase");
                ui.end_row();

                ui.label(egui::RichText::new("No Runtime").strong());
                ui.label("Predictable real-time behavior, no GC pauses");
                ui.end_row();
            });

        ui.add_space(16.0);
        ui.separator();

        ui.heading("Platform Architecture");
        ui.add_space(8.0);

        ui.code(
            r#"
┌──────────────────────────────────────────────────────────────┐
│                   Applications Layer                         │
│  ┌────────────┐  ┌───────────┐  ┌───────────┐  ┌───────────┐ │
│  │r4w-explorer│  │    r4w    │  │ r4w-web   │  │Your App   │ │
│  │   (GUI)    │  │   (CLI)   │  │  (WASM)   │  │           │ │
│  └────────────┘  └───────────┘  └───────────┘  └───────────┘ │
├──────────────────────────────────────────────────────────────┤
│                   Waveform Framework                         │
│   LoRa │ PSK/QAM │ FSK │ OFDM │ DSSS │ AM/FM │ Custom        │
├──────────────────────────────────────────────────────────────┤
│                   Core Libraries                             │
│  r4w-core (DSP) │ r4w-sim (Channels) │ r4w-gui (Widgets)     │
├──────────────────────────────────────────────────────────────┤
│                   Hardware Abstraction                       │
│  SdrDevice Trait │ UDP I/Q Transport │ r4w-fpga              │
└──────────────────────────────────────────────────────────────┘
"#,
        );

        ui.add_space(16.0);
        ui.separator();

        ui.heading("Available Waveforms");
        ui.add_space(8.0);

        egui::Grid::new("waveforms_grid")
            .num_columns(2)
            .spacing([40.0, 4.0])
            .show(ui, |ui| {
                ui.label(egui::RichText::new("Simple").strong());
                ui.label("CW, OOK, PPM");
                ui.end_row();

                ui.label(egui::RichText::new("Analog").strong());
                ui.label("AM, FM");
                ui.end_row();

                ui.label(egui::RichText::new("FSK").strong());
                ui.label("BFSK, 4-FSK");
                ui.end_row();

                ui.label(egui::RichText::new("PSK").strong());
                ui.label("BPSK, QPSK, 8-PSK");
                ui.end_row();

                ui.label(egui::RichText::new("QAM").strong());
                ui.label("16-QAM, 64-QAM, 256-QAM");
                ui.end_row();

                ui.label(egui::RichText::new("Multi-carrier").strong());
                ui.label("OFDM");
                ui.end_row();

                ui.label(egui::RichText::new("Spread Spectrum").strong());
                ui.label("DSSS, FHSS, LoRa (CSS)");
                ui.end_row();

                ui.label(egui::RichText::new("Specialized").strong());
                ui.label("ADS-B, Zigbee, UWB, FMCW");
                ui.end_row();
            });

        ui.add_space(16.0);
        ui.separator();

        ui.heading("FPGA Acceleration");
        ui.add_space(8.0);

        ui.label("R4W supports hardware acceleration:");
        ui.add_space(8.0);

        ui.indent("fpga_targets", |ui| {
            ui.label("• Xilinx Zynq: Primary target (ARM + FPGA SoC)");
            ui.label("• Lattice iCE40/ECP5: Low-cost, open-source toolchain");
            ui.label("• IP Cores: FFT, FIR, chirp generator, DMA");
        });

        ui.add_space(16.0);
        ui.separator();

        ui.heading("Getting Started");
        ui.add_space(8.0);

        ui.code(
            r#"
# Run the GUI explorer
cargo run --bin r4w-explorer

# List available waveforms
cargo run --bin r4w -- waveform --list

# Simulate a LoRa transmission
cargo run --bin r4w -- simulate --message "Hello R4W!" --snr 10.0

# Run in browser (WASM)
cd crates/r4w-web && trunk serve
"#,
        );
    }

    fn render_waveform_content(&self, ui: &mut Ui, waveform_name: &str, params: &LoRaParams) {
        match waveform_name {
            // LoRa modes
            "LoRa" | "LoRa CSS" | "LoRa Mod" | "LoRa Demod" => {
                self.render_lora_overview(ui, params);
            }
            // Simple waveforms
            "CW" => self.render_cw_overview(ui),
            "OOK" => self.render_ook_overview(ui),
            // Pulse modulation
            "PPM" => self.render_ppm_overview(ui),
            "ADS-B" => self.render_adsb_overview(ui),
            // FSK variants
            "BFSK" | "FSK" => self.render_bfsk_overview(ui),
            "4FSK" | "4-FSK" => self.render_4fsk_overview(ui),
            // PSK variants
            "BPSK" => self.render_bpsk_overview(ui),
            "QPSK" => self.render_qpsk_overview(ui),
            "8PSK" | "8-PSK" => self.render_8psk_overview(ui),
            // QAM variants
            "16QAM" | "16-QAM" => self.render_16qam_overview(ui),
            "64QAM" | "64-QAM" => self.render_64qam_overview(ui),
            "256QAM" | "256-QAM" => self.render_256qam_overview(ui),
            // Analog modulation
            "AM" => self.render_am_overview(ui),
            "FM" => self.render_fm_overview(ui),
            // Multi-carrier
            "OFDM" => self.render_ofdm_overview(ui),
            // Spread spectrum
            "DSSS" | "DSSS-QPSK" => self.render_dsss_overview(ui),
            "FHSS" => self.render_fhss_overview(ui),
            // Specialized
            "Zigbee" | "802.15.4" => self.render_zigbee_overview(ui),
            "UWB" => self.render_uwb_overview(ui),
            "FMCW" => self.render_fmcw_overview(ui),
            // Default
            _ => self.render_generic_overview(ui, waveform_name),
        }
    }

    // ==================== LoRa ====================
    fn render_lora_overview(&self, ui: &mut Ui, params: &LoRaParams) {
        ui.heading("LoRa (Long Range)");
        ui.add_space(8.0);

        ui.label(
            "LoRa is a low-power wide-area network (LPWAN) protocol that enables \
            long-range communications at low data rates using Chirp Spread Spectrum (CSS) modulation.",
        );
        ui.add_space(8.0);

        ui.label("Key characteristics:");
        ui.indent("lora_chars", |ui| {
            ui.label("• Range: Up to 10+ km in rural areas");
            ui.label("• Power: Operates at very low power levels");
            ui.label("• Data rate: 0.3 - 50 kbps depending on settings");
            ui.label("• Modulation: Chirp Spread Spectrum (CSS)");
            ui.label("• Frequencies: 868 MHz (EU), 915 MHz (US), 433 MHz");
        });

        ui.add_space(12.0);
        ui.heading("Chirp Spread Spectrum (CSS)");
        ui.add_space(8.0);

        ui.label(
            "LoRa encodes data in chirps - signals whose frequency changes linearly over time. \
            Each symbol is represented by a different starting phase of the chirp.",
        );

        ui.code(
            r#"
  Upchirp (base):       Symbol encoding:
  Frequency              Frequency
      ^                      ^
  fmax|    /             fmax|  /|
      |   /                  | / |  Symbol shifts
      |  /                   |/  |  the chirp start
      | /                    |   /
  fmin|/                 fmin|  /
      +----> Time            +----> Time
"#,
        );

        ui.add_space(8.0);
        ui.label("Why chirps work well:");
        ui.indent("why_chirps", |ui| {
            ui.label("• Excellent noise immunity (processing gain)");
            ui.label("• Resistant to multipath interference");
            ui.label("• Simple FFT-based demodulation");
            ui.label("• Different spreading factors are quasi-orthogonal");
        });

        ui.add_space(12.0);
        ui.heading("Current LoRa Settings");
        ui.add_space(8.0);

        egui::Grid::new("lora_params_grid")
            .num_columns(2)
            .spacing([40.0, 4.0])
            .show(ui, |ui| {
                ui.label("Spreading Factor:");
                ui.label(format!("SF{}", params.sf.value()));
                ui.end_row();

                ui.label("Bandwidth:");
                ui.label(format!("{} kHz", params.bw.hz() / 1000.0));
                ui.end_row();

                ui.label("Coding Rate:");
                ui.label(format!("{}", params.cr));
                ui.end_row();

                ui.label("Symbol Duration:");
                ui.label(format!("{:.3} ms", params.symbol_duration() * 1000.0));
                ui.end_row();

                ui.label("Chips per Symbol:");
                ui.label(format!("{}", params.chips_per_symbol()));
                ui.end_row();

                ui.label("Bit Rate:");
                ui.label(format!("{:.1} bps", params.bit_rate()));
                ui.end_row();
            });

        ui.add_space(12.0);
        ui.heading("LoRa Signal Processing Pipeline");
        ui.add_space(8.0);

        ui.code(
            r#"
TRANSMIT:
  Data ──► Whitening ──► Hamming FEC ──► Interleave ──► Gray Code ──► CSS Mod ──► I/Q
              │              │               │              │            │
           Scramble      Add error       Spread         Adjacent     Chirp
           data          correction      errors         symbols      generation

RECEIVE:
  I/Q ──► Preamble Detect ──► Sync ──► CSS Demod ──► De-Gray ──► De-interleave ──► FEC ──► Data
              │                 │          │
           Find start       CFO/time   FFT-based
                            correction  detection
"#,
        );
    }

    // ==================== CW ====================
    fn render_cw_overview(&self, ui: &mut Ui) {
        ui.heading("Continuous Wave (CW)");
        ui.add_space(8.0);

        ui.label(
            "CW is the simplest form of radio transmission - a pure sinusoidal carrier wave \
            at a single frequency. It's the foundation upon which all other modulation schemes are built.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("cw_apps", |ui| {
            ui.label("• Morse code transmission (on-off keying of CW)");
            ui.label("• Radar systems (continuous wave radar)");
            ui.label("• Test and measurement equipment");
            ui.label("• Frequency reference signals");
            ui.label("• Carrier for other modulation types");
        });

        ui.add_space(12.0);
        ui.heading("Mathematical Representation");
        ui.add_space(8.0);

        ui.code("s(t) = A · cos(2πft + φ)");
        ui.add_space(4.0);
        ui.label("Where: A = amplitude, f = frequency, φ = phase");

        ui.add_space(12.0);
        ui.heading("I/Q Representation");
        ui.add_space(8.0);

        ui.code(
            r#"
  I(t) = A · cos(2πft)    (In-phase)
  Q(t) = A · sin(2πft)    (Quadrature)

  On constellation:
       Q
       ^
       |    * rotates in circle
       |   /
       +--/---> I
          r = A (constant amplitude)
"#,
        );

        ui.add_space(8.0);
        ui.label("Key properties:");
        ui.indent("cw_props", |ui| {
            ui.label("• Constant envelope (amplitude doesn't change)");
            ui.label("• Single spectral line at the carrier frequency");
            ui.label("• No information content without modulation");
            ui.label("• 100% power efficiency (all power in carrier)");
        });
    }

    // ==================== OOK ====================
    fn render_ook_overview(&self, ui: &mut Ui) {
        ui.heading("On-Off Keying (OOK)");
        ui.add_space(8.0);

        ui.label(
            "OOK is the simplest form of digital modulation where the carrier is switched \
            on and off to represent binary 1 and 0. It's essentially amplitude modulation \
            with two levels: full amplitude and zero.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("ook_apps", |ui| {
            ui.label("• Remote controls (garage doors, car key fobs)");
            ui.label("• RFID systems");
            ui.label("• Simple sensor networks");
            ui.label("• Legacy alarm systems");
            ui.label("• Infrared communication (TV remotes)");
        });

        ui.add_space(12.0);
        ui.heading("Signal Representation");
        ui.add_space(8.0);

        ui.code(
            r#"
Binary:    1    0    1    1    0
          ___       ___  ___
Signal:  |   |     |   ||   |
         |   |_____|   ||   |_____

         ON  OFF   ON   ON  OFF
"#,
        );

        ui.add_space(12.0);
        ui.heading("Processing Pipeline");
        ui.add_space(8.0);

        ui.code(
            r#"
TRANSMIT:
  Bits ──► Symbol Map ──► Carrier Gate ──► I/Q Samples
              │               │
           0→OFF          Multiply by
           1→ON           carrier

RECEIVE:
  I/Q ──► Envelope Detect ──► Threshold ──► Bits
              │                   │
           |sample|           Compare to
           magnitude          decision level
"#,
        );

        ui.add_space(8.0);
        ui.label("Characteristics:");
        ui.indent("ook_chars", |ui| {
            ui.label("• Spectral efficiency: 1 bit/symbol");
            ui.label("• Simple implementation (just a switch)");
            ui.label("• Poor noise immunity compared to FSK/PSK");
            ui.label("• Non-constant envelope (affects PA efficiency)");
            ui.label("• Bandwidth ≈ 2 × bit rate");
        });
    }

    // ==================== PPM ====================
    fn render_ppm_overview(&self, ui: &mut Ui) {
        ui.heading("Pulse Position Modulation (PPM)");
        ui.add_space(8.0);

        ui.label(
            "PPM encodes information in the timing (position) of pulses within a symbol period. \
            Unlike OOK which uses pulse presence/absence, PPM shifts the pulse position to \
            represent different data values.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("ppm_apps", |ui| {
            ui.label("• Optical communication (fiber optics, free-space)");
            ui.label("• Infrared data transmission");
            ui.label("• Ultra-wideband (UWB) systems");
            ui.label("• Remote controls");
            ui.label("• Servo control signals (RC hobby)");
        });

        ui.add_space(12.0);
        ui.heading("Binary PPM Encoding");
        ui.add_space(8.0);

        ui.code(
            r#"
Symbol period divided into 2 slots:

Bit 0:  |▓▓▓▓|    |     Bit 1:  |    |▓▓▓▓|
        early slot              late slot

Time:   |-- T/2 --|-- T/2 --|

Position determines bit value, not presence
"#,
        );

        ui.add_space(12.0);
        ui.heading("Processing Pipeline");
        ui.add_space(8.0);

        ui.code(
            r#"
TRANSMIT:
  Bits ──► Position Encoder ──► Pulse Generator ──► I/Q
              │                     │
           Map bit to           Generate pulse
           time slot            at position

RECEIVE:
  I/Q ──► Matched Filter ──► Peak Detect ──► Position Decode ──► Bits
              │                  │                │
           Correlate         Find pulse       Compare to
           with pulse        timing           threshold
"#,
        );

        ui.add_space(8.0);
        ui.label("Advantages:");
        ui.indent("ppm_adv", |ui| {
            ui.label("• Constant average power (good for optical)");
            ui.label("• Better power efficiency than OOK");
            ui.label("• Simple threshold-based detection");
            ui.label("• M-ary PPM can encode log₂(M) bits per symbol");
        });
    }

    // ==================== ADS-B ====================
    fn render_adsb_overview(&self, ui: &mut Ui) {
        ui.heading("ADS-B (Automatic Dependent Surveillance-Broadcast)");
        ui.add_space(8.0);

        ui.label(
            "ADS-B is an aviation surveillance technology where aircraft broadcast their \
            identification, position, altitude, and velocity. It uses PPM encoding at 1090 MHz.",
        );

        ui.add_space(8.0);
        ui.label("Message content:");
        ui.indent("adsb_content", |ui| {
            ui.label("• Aircraft identification (ICAO 24-bit address)");
            ui.label("• Position (latitude, longitude, altitude)");
            ui.label("• Velocity (ground speed, heading, vertical rate)");
            ui.label("• Aircraft category and call sign");
            ui.label("• Emergency/priority status");
        });

        ui.add_space(12.0);
        ui.heading("Signal Structure");
        ui.add_space(8.0);

        ui.code(
            r#"
1090 MHz Mode S Extended Squitter:

|--8μs preamble--|--------112 bit data (56μs)--------|

Preamble:  _   _             Data uses Manchester/PPM:
          | | | |            Bit 1: |▓|_|  (high-low)
          | | | |            Bit 0: |_|▓|  (low-high)
          |_| |_|____

Each bit: 1μs total (0.5μs per chip)
Data rate: 1 Mbps
"#,
        );

        ui.add_space(12.0);
        ui.heading("Message Types (Downlink Format)");
        ui.add_space(8.0);

        ui.indent("adsb_types", |ui| {
            ui.label("• DF17: Extended Squitter (position, velocity, ID)");
            ui.label("• DF18: TIS-B/ADS-R messages");
            ui.label("• DF4/5/20/21: Altitude and identity replies");
            ui.label("• DF11: All-call reply");
        });

        ui.add_space(8.0);
        ui.label("Characteristics:");
        ui.indent("adsb_chars", |ui| {
            ui.label("• Frequency: 1090 MHz");
            ui.label("• Modulation: PPM (Manchester encoding)");
            ui.label("• Data rate: 1 Mbps");
            ui.label("• Range: ~250 nautical miles line-of-sight");
            ui.label("• Update rate: ~1 Hz per aircraft");
        });
    }

    // ==================== BFSK ====================
    fn render_bfsk_overview(&self, ui: &mut Ui) {
        ui.heading("Binary Frequency Shift Keying (BFSK)");
        ui.add_space(8.0);

        ui.label(
            "BFSK encodes binary data by switching between two carrier frequencies. \
            A '0' is transmitted as frequency f₁ and a '1' as frequency f₂, where \
            the separation is called the frequency deviation.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("bfsk_apps", |ui| {
            ui.label("• Caller ID signaling");
            ui.label("• Paging systems");
            ui.label("• Low-rate telemetry");
            ui.label("• RFID systems");
            ui.label("• Early modems (Bell 103: 300 baud)");
        });

        ui.add_space(12.0);
        ui.heading("Signal Representation");
        ui.add_space(8.0);

        ui.code(
            r#"
Binary:    0         1         0

Frequency:
    f₂    --------  ~~~~~~~~  --------
    fc    ........  ........  ........
    f₁    ~~~~~~~~  --------  ~~~~~~~~

    f₁ = fc - Δf    f₂ = fc + Δf

Deviation (Δf) and symbol rate determine bandwidth
"#,
        );

        ui.add_space(12.0);
        ui.heading("Modulation Index");
        ui.add_space(8.0);

        ui.code("h = 2Δf / symbol_rate");
        ui.add_space(4.0);
        ui.label("The modulation index affects spectral efficiency:");
        ui.indent("bfsk_h", |ui| {
            ui.label("• h = 0.5: MSK (Minimum Shift Keying) - most compact");
            ui.label("• h = 1.0: Sunde's FSK - orthogonal frequencies");
            ui.label("• h > 1.0: Wide deviation - easier detection");
        });

        ui.add_space(12.0);
        ui.heading("Processing Pipeline");
        ui.add_space(8.0);

        ui.code(
            r#"
TRANSMIT:
  Bits ──► Freq Map ──► VCO/NCO ──► I/Q Samples
              │            │
           0→f₁        Generate
           1→f₂        sinusoid

RECEIVE:
  I/Q ──► Matched Filters ──► Compare ──► Bits
              │                  │
           f₁ filter         Which filter
           f₂ filter         has more energy?
"#,
        );
    }

    // ==================== 4-FSK ====================
    fn render_4fsk_overview(&self, ui: &mut Ui) {
        ui.heading("4-Level Frequency Shift Keying (4-FSK)");
        ui.add_space(8.0);

        ui.label(
            "4-FSK extends BFSK by using four distinct frequencies, allowing 2 bits to be \
            transmitted per symbol. This doubles the spectral efficiency compared to BFSK.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("4fsk_apps", |ui| {
            ui.label("• DMR (Digital Mobile Radio)");
            ui.label("• dPMR (digital Private Mobile Radio)");
            ui.label("• NXDN digital radio");
            ui.label("• APCO P25 Phase 2");
            ui.label("• Industrial telemetry");
        });

        ui.add_space(12.0);
        ui.heading("Symbol Mapping");
        ui.add_space(8.0);

        ui.code(
            r#"
4 frequencies for 4 symbols (2 bits each):

Frequency:
    f₃ (+3Δf)  ────  Symbol 11 (3)
    f₂ (+1Δf)  ────  Symbol 10 (2)
    f₁ (-1Δf)  ────  Symbol 01 (1)
    f₀ (-3Δf)  ────  Symbol 00 (0)

Typically uses Gray coding:
  00 → f₀, 01 → f₁, 11 → f₂, 10 → f₃
  (Adjacent frequencies differ by 1 bit)
"#,
        );

        ui.add_space(12.0);
        ui.heading("Spectral Efficiency");
        ui.add_space(8.0);

        ui.indent("4fsk_eff", |ui| {
            ui.label("• Bits per symbol: 2");
            ui.label("• Data rate = 2 × symbol rate");
            ui.label("• Requires ~4× bandwidth of BFSK for same symbol rate");
            ui.label("• But achieves 2× throughput");
        });

        ui.add_space(8.0);
        ui.label("Trade-offs vs BFSK:");
        ui.indent("4fsk_trade", |ui| {
            ui.label("• Pro: Higher data rate for given symbol rate");
            ui.label("• Pro: Same constant envelope (good PA efficiency)");
            ui.label("• Con: Requires higher SNR for same BER");
            ui.label("• Con: More complex receiver (4 matched filters)");
        });
    }

    // ==================== BPSK ====================
    fn render_bpsk_overview(&self, ui: &mut Ui) {
        ui.heading("Binary Phase Shift Keying (BPSK)");
        ui.add_space(8.0);

        ui.label(
            "BPSK encodes binary data by changing the phase of the carrier between two states \
            180° apart. It's the simplest form of PSK and provides the best noise immunity \
            among PSK variants.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("bpsk_apps", |ui| {
            ui.label("• Deep space communication (robust against noise)");
            ui.label("• GPS signal spreading");
            ui.label("• CDMA pilot channels");
            ui.label("• Cable modems (upstream)");
            ui.label("• RFID systems");
        });

        ui.add_space(12.0);
        ui.heading("Constellation Diagram");
        ui.add_space(8.0);

        ui.code(
            r#"
         Q
         ^
         |
    0────+────1    Two points on I-axis
   (-1)  |   (+1)  180° phase difference
         |
       ──┴──> I

Bit 0: Phase = 180° → I = -1, Q = 0
Bit 1: Phase = 0°   → I = +1, Q = 0
"#,
        );

        ui.add_space(12.0);
        ui.heading("Processing Pipeline");
        ui.add_space(8.0);

        ui.code(
            r#"
TRANSMIT:
  Bits ──► NRZ Map ──► Multiply Carrier ──► I/Q
              │              │
           0 → -1        s(t) = d(t)·cos(2πfct)
           1 → +1

RECEIVE:
  I/Q ──► Carrier Sync ──► Correlate ──► Threshold ──► Bits
              │               │              │
           Recover        Integrate      Sign of
           phase          over symbol    correlation
"#,
        );

        ui.add_space(8.0);
        ui.label("Key properties:");
        ui.indent("bpsk_props", |ui| {
            ui.label("• Spectral efficiency: 1 bit/symbol");
            ui.label("• Most robust PSK (maximum phase separation)");
            ui.label("• Constant envelope");
            ui.label("• Requires coherent detection (carrier recovery)");
            ui.label("• Eb/N0 for BER=10⁻⁵: ~9.6 dB");
        });
    }

    // ==================== QPSK ====================
    fn render_qpsk_overview(&self, ui: &mut Ui) {
        ui.heading("Quadrature Phase Shift Keying (QPSK)");
        ui.add_space(8.0);

        ui.label(
            "QPSK uses four phase states separated by 90° to encode 2 bits per symbol. \
            It's widely used because it doubles BPSK's throughput while maintaining \
            a constant envelope.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("qpsk_apps", |ui| {
            ui.label("• Satellite communication (DVB-S, DVB-S2)");
            ui.label("• Cable modems (DOCSIS)");
            ui.label("• LTE/5G (for control channels)");
            ui.label("• Wi-Fi (802.11 at low rates)");
            ui.label("• Digital video broadcasting");
        });

        ui.add_space(12.0);
        ui.heading("Constellation Diagram");
        ui.add_space(8.0);

        ui.code(
            r#"
         Q
         ^
    01   |   11     4 points at 45°, 135°, 225°, 315°
      *  |  *       Each encodes 2 bits (dibit)
         |
   ──────+──────> I
         |
      *  |  *
    00   |   10

Gray coding: Adjacent points differ by 1 bit
"#,
        );

        ui.add_space(12.0);
        ui.heading("Mathematical Representation");
        ui.add_space(8.0);

        ui.code(
            r#"
s(t) = I(t)·cos(2πfct) - Q(t)·sin(2πfct)

Where I(t), Q(t) ∈ {-1, +1} for each symbol

Dibits → (I, Q):
  00 → (-1, -1)    01 → (-1, +1)
  10 → (+1, -1)    11 → (+1, +1)
"#,
        );

        ui.add_space(8.0);
        ui.label("Key properties:");
        ui.indent("qpsk_props", |ui| {
            ui.label("• Spectral efficiency: 2 bits/symbol");
            ui.label("• Same bandwidth as BPSK, 2× throughput");
            ui.label("• Same BER vs Eb/N0 as BPSK");
            ui.label("• Constant envelope (like BPSK)");
            ui.label("• Requires coherent detection");
        });
    }

    // ==================== 8-PSK ====================
    fn render_8psk_overview(&self, ui: &mut Ui) {
        ui.heading("8-Phase Shift Keying (8-PSK)");
        ui.add_space(8.0);

        ui.label(
            "8-PSK uses eight equally-spaced phase states to encode 3 bits per symbol. \
            It increases throughput over QPSK but requires higher SNR due to closer \
            constellation points.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("8psk_apps", |ui| {
            ui.label("• Satellite communication (DVB-S2 adaptive)");
            ui.label("• EDGE (Enhanced Data GSM Evolution)");
            ui.label("• Digital microwave links");
            ui.label("• Military communications");
        });

        ui.add_space(12.0);
        ui.heading("Constellation Diagram");
        ui.add_space(8.0);

        ui.code(
            r#"
            Q
            ^
       010  |  011
         *  |  *
    001  *  |     * 100
            |
   ─────────+─────────> I
            |
    000  *  |     * 101
         *  |  *
       111  |  110

8 points at 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°
"#,
        );

        ui.add_space(8.0);
        ui.label("Trade-offs vs QPSK:");
        ui.indent("8psk_trade", |ui| {
            ui.label("• Pro: 50% higher throughput (3 bits vs 2 bits)");
            ui.label("• Pro: Same bandwidth as QPSK");
            ui.label("• Con: ~3-4 dB higher SNR required for same BER");
            ui.label("• Con: More susceptible to phase noise");
        });

        ui.add_space(8.0);
        ui.label("Gray coding is essential:");
        ui.indent("8psk_gray", |ui| {
            ui.label("• Adjacent symbols differ by only 1 bit");
            ui.label("• Most errors are to adjacent symbols");
            ui.label("• Minimizes bit errors from symbol errors");
        });
    }

    // ==================== 16-QAM ====================
    fn render_16qam_overview(&self, ui: &mut Ui) {
        ui.heading("16-Quadrature Amplitude Modulation (16-QAM)");
        ui.add_space(8.0);

        ui.label(
            "16-QAM combines phase and amplitude modulation to create 16 distinct symbols, \
            encoding 4 bits per symbol. It arranges constellation points in a square grid, \
            offering a good balance between efficiency and robustness.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("16qam_apps", |ui| {
            ui.label("• LTE/5G (adaptive modulation)");
            ui.label("• Wi-Fi 802.11a/g/n/ac/ax");
            ui.label("• Digital cable TV");
            ui.label("• Microwave backhaul");
            ui.label("• Digital radio (DAB+)");
        });

        ui.add_space(12.0);
        ui.heading("Constellation Diagram");
        ui.add_space(8.0);

        ui.code(
            r#"
         Q
         ^
    0010 0110 1110 1010    4×4 grid = 16 points
      *    *    *    *     Each point = 4 bits
    0011 0111 1111 1011
      *    *    *    *
   ──────────────────────> I
    0001 0101 1101 1001
      *    *    *    *
    0000 0100 1100 1000
      *    *    *    *

I and Q each carry 2 bits independently
"#,
        );

        ui.add_space(12.0);
        ui.heading("Key Characteristics");
        ui.add_space(8.0);

        ui.indent("16qam_chars", |ui| {
            ui.label("• Spectral efficiency: 4 bits/symbol");
            ui.label("• Non-constant envelope (varies with symbol)");
            ui.label("• Requires linear power amplifier");
            ui.label("• More susceptible to noise than QPSK");
            ui.label("• Required SNR ~4-5 dB higher than QPSK");
        });

        ui.add_space(8.0);
        ui.label("Why square constellation?");
        ui.indent("16qam_why", |ui| {
            ui.label("• I and Q can be processed independently");
            ui.label("• Simpler encoding/decoding logic");
            ui.label("• Gray coding is straightforward");
            ui.label("• Near-optimal for Gaussian noise");
        });
    }

    // ==================== 64-QAM ====================
    fn render_64qam_overview(&self, ui: &mut Ui) {
        ui.heading("64-Quadrature Amplitude Modulation (64-QAM)");
        ui.add_space(8.0);

        ui.label(
            "64-QAM uses an 8×8 grid of constellation points to encode 6 bits per symbol. \
            It's commonly used in high-throughput systems where channel conditions allow.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("64qam_apps", |ui| {
            ui.label("• Digital cable TV (primary modulation)");
            ui.label("• Wi-Fi 802.11ac/ax");
            ui.label("• LTE/5G (good channel conditions)");
            ui.label("• DOCSIS cable modems");
            ui.label("• DVB-C (cable broadcast)");
        });

        ui.add_space(12.0);
        ui.heading("Constellation Structure");
        ui.add_space(8.0);

        ui.code(
            r#"
         Q
         ^
    ─────┼─────    8×8 = 64 points
    *****│*****    6 bits per symbol
    *****│*****
    *****│*****    I levels: -7, -5, -3, -1, +1, +3, +5, +7
   ──────┼──────> I Q levels: same
    *****│*****
    *****│*****    Peak-to-average power ratio
    *****│*****    higher than 16-QAM
    ─────┼─────
"#,
        );

        ui.add_space(8.0);
        ui.label("Performance comparison:");
        ui.indent("64qam_perf", |ui| {
            ui.label("• vs 16-QAM: +50% throughput, ~6 dB more SNR needed");
            ui.label("• vs QPSK: +200% throughput, ~10 dB more SNR needed");
            ui.label("• Minimum distance between points is smaller");
            ui.label("• More sensitive to phase and amplitude errors");
        });
    }

    // ==================== 256-QAM ====================
    fn render_256qam_overview(&self, ui: &mut Ui) {
        ui.heading("256-Quadrature Amplitude Modulation (256-QAM)");
        ui.add_space(8.0);

        ui.label(
            "256-QAM is a high-order modulation using a 16×16 grid to encode 8 bits (1 byte) \
            per symbol. It achieves very high spectral efficiency but requires excellent \
            channel conditions.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("256qam_apps", |ui| {
            ui.label("• DOCSIS 3.1 cable modems");
            ui.label("• Wi-Fi 802.11ac/ax (excellent conditions)");
            ui.label("• DVB-C2 cable broadcast");
            ui.label("• Point-to-point microwave (short hops)");
            ui.label("• 5G NR (ideal conditions)");
        });

        ui.add_space(12.0);
        ui.heading("SNR Requirements");
        ui.add_space(8.0);

        ui.code(
            r#"
Modulation    Bits/sym    Approx SNR for BER=10⁻⁵
─────────────────────────────────────────────────
QPSK             2            ~10 dB
16-QAM           4            ~17 dB
64-QAM           6            ~23 dB
256-QAM          8            ~29 dB
─────────────────────────────────────────────────
Each doubling of points adds ~6 dB requirement
"#,
        );

        ui.add_space(8.0);
        ui.label("Challenges:");
        ui.indent("256qam_challenges", |ui| {
            ui.label("• Very tight phase and amplitude tolerance");
            ui.label("• High peak-to-average power ratio (PAPR)");
            ui.label("• Requires very linear amplifiers");
            ui.label("• Sensitive to I/Q imbalance");
            ui.label("• Needs excellent carrier synchronization");
        });
    }

    // ==================== AM ====================
    fn render_am_overview(&self, ui: &mut Ui) {
        ui.heading("Amplitude Modulation (AM)");
        ui.add_space(8.0);

        ui.label(
            "AM is one of the oldest modulation techniques where the amplitude of a carrier \
            is varied in proportion to the message signal. Though less efficient than FM, \
            it remains important for broadcasting and is the basis for QAM.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("am_apps", |ui| {
            ui.label("• AM radio broadcasting (530-1700 kHz)");
            ui.label("• Aviation communication (VHF AM)");
            ui.label("• Citizens Band (CB) radio");
            ui.label("• Amateur radio");
            ui.label("• Two-way radio systems");
        });

        ui.add_space(12.0);
        ui.heading("Mathematical Representation");
        ui.add_space(8.0);

        ui.code(
            r#"
s(t) = [A + m(t)] · cos(2πfct)

Where:
  A = carrier amplitude
  m(t) = message signal
  fc = carrier frequency

Modulation index: μ = max|m(t)| / A
  μ < 1: No overmodulation
  μ = 1: 100% modulation (carrier goes to zero)
  μ > 1: Overmodulation (distortion)
"#,
        );

        ui.add_space(12.0);
        ui.heading("AM Variants");
        ui.add_space(8.0);

        ui.indent("am_variants", |ui| {
            ui.label("• DSB-FC: Double Sideband Full Carrier (standard AM)");
            ui.label("• DSB-SC: Double Sideband Suppressed Carrier");
            ui.label("• SSB: Single Sideband (USB or LSB)");
            ui.label("• VSB: Vestigial Sideband");
        });

        ui.add_space(8.0);
        ui.label("Efficiency considerations:");
        ui.indent("am_eff", |ui| {
            ui.label("• At 100% modulation, only 33% power in sidebands");
            ui.label("• 67% of power wasted in carrier");
            ui.label("• SSB is most efficient (removes carrier + one sideband)");
            ui.label("• But DSB-FC allows simple envelope detection");
        });
    }

    // ==================== FM ====================
    fn render_fm_overview(&self, ui: &mut Ui) {
        ui.heading("Frequency Modulation (FM)");
        ui.add_space(8.0);

        ui.label(
            "FM encodes information by varying the carrier frequency in proportion to the \
            message signal. It provides excellent noise immunity and is the basis for \
            high-fidelity audio broadcasting.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("fm_apps", |ui| {
            ui.label("• FM radio broadcasting (88-108 MHz)");
            ui.label("• Television audio (analog)");
            ui.label("• Two-way radio (VHF/UHF)");
            ui.label("• Satellite communication");
            ui.label("• Analog cellular (AMPS)");
        });

        ui.add_space(12.0);
        ui.heading("Mathematical Representation");
        ui.add_space(8.0);

        ui.code(
            r#"
s(t) = A · cos(2πfct + 2πkf ∫m(τ)dτ)

Instantaneous frequency:
  f(t) = fc + kf · m(t)

Where:
  kf = frequency sensitivity (Hz/volt)
  Δf = peak frequency deviation = kf · max|m(t)|
"#,
        );

        ui.add_space(12.0);
        ui.heading("Modulation Index");
        ui.add_space(8.0);

        ui.code(
            r#"
β = Δf / fm_max   (fm_max = highest modulating frequency)

β < 1:  Narrowband FM (similar bandwidth to AM)
β > 1:  Wideband FM (better noise performance)

Carson's Rule for bandwidth:
  BW ≈ 2(Δf + fm_max) = 2fm_max(β + 1)
"#,
        );

        ui.add_space(8.0);
        ui.label("Key advantages:");
        ui.indent("fm_adv", |ui| {
            ui.label("• Constant envelope (efficient power amplifiers)");
            ui.label("• Excellent noise immunity (capture effect)");
            ui.label("• No amplitude distortion from multipath");
            ui.label("• Can trade bandwidth for SNR improvement");
        });
    }

    // ==================== OFDM ====================
    fn render_ofdm_overview(&self, ui: &mut Ui) {
        ui.heading("Orthogonal Frequency Division Multiplexing (OFDM)");
        ui.add_space(8.0);

        ui.label(
            "OFDM divides a wideband channel into many narrow orthogonal subcarriers, \
            each carrying data at a lower rate. This makes it highly resistant to \
            multipath fading and enables efficient use of spectrum.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("ofdm_apps", |ui| {
            ui.label("• Wi-Fi (802.11a/g/n/ac/ax)");
            ui.label("• LTE/5G NR (downlink)");
            ui.label("• DVB-T/T2 (digital TV)");
            ui.label("• DAB/DAB+ (digital radio)");
            ui.label("• DSL (ADSL, VDSL)");
            ui.label("• WiMAX");
        });

        ui.add_space(12.0);
        ui.heading("Key Concept: Orthogonality");
        ui.add_space(8.0);

        ui.code(
            r#"
Frequency domain:
    ▲
    │  ╱╲   ╱╲   ╱╲   ╱╲   Each subcarrier is a sinc function
    │ ╱  ╲ ╱  ╲ ╱  ╲ ╱  ╲  Peaks align with zeros of neighbors
    │╱    ╳    ╳    ╳    ╲ → No inter-carrier interference (ICI)
    ├─────┴────┴────┴─────► f
    f₀   f₁   f₂   f₃   f₄

Subcarrier spacing: Δf = 1/T_symbol
"#,
        );

        ui.add_space(12.0);
        ui.heading("OFDM Symbol Structure");
        ui.add_space(8.0);

        ui.code(
            r#"
|←────── OFDM Symbol ──────→|

|←─ CP ─→|←──── Data ────→|
  Cyclic     FFT period
  Prefix     (N samples)

CP copies end of symbol to beginning
→ Prevents ISI between symbols
→ Makes channel appear circular (enables FFT processing)
"#,
        );

        ui.add_space(8.0);
        ui.label("Advantages:");
        ui.indent("ofdm_adv", |ui| {
            ui.label("• Resistant to frequency-selective fading");
            ui.label("• Simple equalization (1 tap per subcarrier)");
            ui.label("• Efficient implementation via FFT/IFFT");
            ui.label("• Flexible subcarrier allocation");
            ui.label("• Supports adaptive modulation per subcarrier");
        });

        ui.add_space(8.0);
        ui.label("Challenges:");
        ui.indent("ofdm_chal", |ui| {
            ui.label("• High peak-to-average power ratio (PAPR)");
            ui.label("• Sensitive to frequency offset");
            ui.label("• Requires precise synchronization");
            ui.label("• Cyclic prefix reduces efficiency");
        });
    }

    // ==================== DSSS ====================
    fn render_dsss_overview(&self, ui: &mut Ui) {
        ui.heading("Direct Sequence Spread Spectrum (DSSS)");
        ui.add_space(8.0);

        ui.label(
            "DSSS spreads the signal over a wider bandwidth by multiplying with a \
            high-rate pseudo-noise (PN) sequence. This provides processing gain, \
            interference rejection, and enables multiple access.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("dsss_apps", |ui| {
            ui.label("• GPS (all satellite signals)");
            ui.label("• CDMA cellular (IS-95, CDMA2000)");
            ui.label("• Wi-Fi 802.11b (Barker code)");
            ui.label("• Zigbee/802.15.4");
            ui.label("• Military LPI/LPD communications");
        });

        ui.add_space(12.0);
        ui.heading("Spreading Process");
        ui.add_space(8.0);

        ui.code(
            r#"
Data:     |  1  |  0  |  1  |   (slow rate)
           ─────  ─────  ─────

PN Code:  |+-+-+|+-+-+|+-+-+|   (fast rate, N chips/bit)
           ▓░▓░▓ ▓░▓░▓ ▓░▓░▓

Spread:   |+-+-+|-+-+-|+-+-+|   Data XOR PN code
           ▓░▓░▓ ░▓░▓░ ▓░▓░▓

Processing Gain = 10·log₁₀(N) dB
   where N = chips per bit
"#,
        );

        ui.add_space(12.0);
        ui.heading("Despreading and Processing Gain");
        ui.add_space(8.0);

        ui.code(
            r#"
At receiver:
  Received signal × same PN code → Original data

Narrowband interference:
  Interference × PN code → Spread (reduced power/Hz)

CDMA Multiple Access:
  Each user has unique PN code
  Cross-correlation is low → Users appear as noise
"#,
        );

        ui.add_space(8.0);
        ui.label("PN Sequence properties:");
        ui.indent("dsss_pn", |ui| {
            ui.label("• Appears random but is deterministic");
            ui.label("• Sharp autocorrelation peak");
            ui.label("• Low cross-correlation between codes");
            ui.label("• Common: Gold codes, M-sequences, Kasami");
        });
    }

    // ==================== FHSS ====================
    fn render_fhss_overview(&self, ui: &mut Ui) {
        ui.heading("Frequency Hopping Spread Spectrum (FHSS)");
        ui.add_space(8.0);

        ui.label(
            "FHSS spreads the signal by rapidly switching the carrier frequency \
            according to a pseudo-random hopping pattern. This provides resistance \
            to narrowband interference and jamming.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("fhss_apps", |ui| {
            ui.label("• Bluetooth (79 channels, 1600 hops/sec)");
            ui.label("• Military radios (anti-jamming)");
            ui.label("• Wireless security systems");
            ui.label("• Some cordless phones");
            ui.label("• LoRa (optional frequency hopping mode)");
        });

        ui.add_space(12.0);
        ui.heading("Hopping Pattern");
        ui.add_space(8.0);

        ui.code(
            r#"
Frequency
    ^
f₇  │    ▓▓▓▓
f₆  │                    ▓▓▓▓
f₅  │        ▓▓▓▓
f₄  │                            ▓▓▓▓
f₃  │                ▓▓▓▓
f₂  │▓▓▓▓
f₁  │            ▓▓▓▓
    └────────────────────────────────► Time
         Hop period (dwell time)

Pattern: f₂→f₇→f₅→f₁→f₃→f₆→f₄→...
Pattern known only to TX and RX
"#,
        );

        ui.add_space(12.0);
        ui.heading("Fast vs Slow Hopping");
        ui.add_space(8.0);

        ui.indent("fhss_types", |ui| {
            ui.label("Fast hopping: Multiple hops per symbol");
            ui.label("  → Better interference averaging");
            ui.label("  → More complex synchronization");
            ui.label("");
            ui.label("Slow hopping: Multiple symbols per hop");
            ui.label("  → Simpler implementation");
            ui.label("  → Used in Bluetooth, 802.11 FHSS");
        });

        ui.add_space(8.0);
        ui.label("Advantages:");
        ui.indent("fhss_adv", |ui| {
            ui.label("• Resistant to narrowband interference");
            ui.label("• Hard to detect and jam");
            ui.label("• Multiple users can share band (CDMA-like)");
            ui.label("• Avoids multipath on any single frequency");
        });
    }

    // ==================== Zigbee ====================
    fn render_zigbee_overview(&self, ui: &mut Ui) {
        ui.heading("Zigbee / IEEE 802.15.4");
        ui.add_space(8.0);

        ui.label(
            "Zigbee is a low-power, low-data-rate wireless protocol designed for \
            IoT and sensor networks. It uses O-QPSK with DSSS spreading for robust \
            communication in noisy environments.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("zigbee_apps", |ui| {
            ui.label("• Smart home automation (lights, thermostats)");
            ui.label("• Industrial sensors and monitoring");
            ui.label("• Building automation");
            ui.label("• Healthcare monitoring");
            ui.label("• Smart metering");
        });

        ui.add_space(12.0);
        ui.heading("PHY Layer Characteristics");
        ui.add_space(8.0);

        ui.code(
            r#"
2.4 GHz Band (16 channels):
─────────────────────────────────────────
Modulation:      O-QPSK (Offset QPSK)
Chip rate:       2 Mchips/sec
Symbol rate:     62.5 ksymbols/sec
Data rate:       250 kbps
Spreading:       32 chips per symbol (4 bits)
Processing gain: 10·log₁₀(32/4) = 9 dB
─────────────────────────────────────────
"#,
        );

        ui.add_space(12.0);
        ui.heading("Symbol-to-Chip Mapping");
        ui.add_space(8.0);

        ui.code(
            r#"
4 data bits → 32-chip sequence

Example (simplified):
  0000 → chip sequence A (32 chips)
  0001 → chip sequence B (32 chips)
  ...
  1111 → chip sequence P (32 chips)

16 possible symbols, each with unique 32-chip code
Chips transmitted with O-QPSK modulation
"#,
        );

        ui.add_space(8.0);
        ui.label("O-QPSK advantages:");
        ui.indent("zigbee_oqpsk", |ui| {
            ui.label("• Maximum 90° phase transitions (not 180°)");
            ui.label("• Reduced envelope variation");
            ui.label("• Better spectral containment");
            ui.label("• Simpler power amplifier requirements");
        });
    }

    // ==================== UWB ====================
    fn render_uwb_overview(&self, ui: &mut Ui) {
        ui.heading("Ultra-Wideband (UWB) Impulse Radio");
        ui.add_space(8.0);

        ui.label(
            "UWB transmits information using very short pulses (nanoseconds) that spread \
            energy across a wide bandwidth (>500 MHz). This enables precise ranging, \
            high data rates, and coexistence with narrowband systems.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("uwb_apps", |ui| {
            ui.label("• Indoor positioning (Apple AirTag, Samsung SmartTag)");
            ui.label("• Automotive keyless entry");
            ui.label("• Asset tracking");
            ui.label("• High-speed WPAN (obsolete)");
            ui.label("• Through-wall radar");
        });

        ui.add_space(12.0);
        ui.heading("Pulse Types");
        ui.add_space(8.0);

        ui.code(
            r#"
Gaussian Monocycle:        Gaussian Doublet:
        ▲                         ▲
      ╱ ╲                       ╱ ╲
     ╱   ╲                     ╱   ╲
────╱─────╲────           ───╱─────╲───
           ╲ ╱                      ╲ ╱
            v                        v

Pulse duration: 0.2-2 nanoseconds
Bandwidth: 500 MHz - 7.5 GHz (FCC: 3.1-10.6 GHz)
"#,
        );

        ui.add_space(12.0);
        ui.heading("Modulation Schemes");
        ui.add_space(8.0);

        ui.indent("uwb_mod", |ui| {
            ui.label("• OOK: Pulse present (1) or absent (0)");
            ui.label("• BPSK: Pulse polarity (+1 or -1)");
            ui.label("• PPM: Pulse position encodes data");
            ui.label("• Hybrid: Combination for higher rates");
        });

        ui.add_space(8.0);
        ui.label("Key characteristics:");
        ui.indent("uwb_chars", |ui| {
            ui.label("• Very low power spectral density (<-41.3 dBm/MHz)");
            ui.label("• Coexists with GPS, WiFi, cellular");
            ui.label("• Excellent multipath resolution (cm-level)");
            ui.label("• Ranging accuracy: ~10 cm");
            ui.label("• High immunity to interference");
        });
    }

    // ==================== FMCW ====================
    fn render_fmcw_overview(&self, ui: &mut Ui) {
        ui.heading("FMCW (Frequency Modulated Continuous Wave) Radar");
        ui.add_space(8.0);

        ui.label(
            "FMCW is a radar technique that transmits a continuous signal with linearly \
            changing frequency (chirp). By mixing the received echo with the transmitted \
            signal, range and velocity information can be extracted.",
        );

        ui.add_space(8.0);
        ui.label("Applications:");
        ui.indent("fmcw_apps", |ui| {
            ui.label("• Automotive radar (adaptive cruise, collision avoidance)");
            ui.label("• Industrial level measurement");
            ui.label("• Aircraft altimeters");
            ui.label("• Speed guns");
            ui.label("• Weather radar");
        });

        ui.add_space(12.0);
        ui.heading("Chirp and Beat Frequency");
        ui.add_space(8.0);

        ui.code(
            r#"
Frequency
    ^
    │     ╱╲      ╱╲      TX chirp
fmax│    ╱  ╲    ╱  ╲
    │   ╱    ╲  ╱    ╲
    │  ╱......╲╱......╲   RX echo (delayed)
fmin│ ╱
    └─────────────────────► Time
      |←─ Δt ─→|
       delay = 2R/c

Beat frequency: fb = (Δf/T_chirp) × Δt = (2BW×R)/(c×T)
"#,
        );

        ui.add_space(12.0);
        ui.heading("Range and Velocity Resolution");
        ui.add_space(8.0);

        ui.code(
            r#"
Range resolution:    ΔR = c / (2 × BW)
  BW = 150 MHz → ΔR = 1.0 m
  BW = 4 GHz   → ΔR = 3.75 cm

Velocity resolution: Δv = λ / (2 × T_frame)
  (Measured via Doppler across multiple chirps)

Maximum range: Limited by ADC sampling rate
Maximum velocity: Limited by chirp repetition rate
"#,
        );

        ui.add_space(8.0);
        ui.label("Chirp patterns:");
        ui.indent("fmcw_patterns", |ui| {
            ui.label("• Sawtooth up: Simple, common for ranging");
            ui.label("• Sawtooth down: Same as up, opposite slope");
            ui.label("• Triangle: Up+down enables velocity sign detection");
            ui.label("• Multiple slopes: Resolves range-Doppler ambiguity");
        });
    }

    // ==================== Generic fallback ====================
    fn render_generic_overview(&self, ui: &mut Ui, waveform_name: &str) {
        ui.heading(format!("{} Overview", waveform_name));
        ui.add_space(8.0);

        ui.label(format!(
            "This is the {} waveform. Use the Waveform Lab view to explore \
            its characteristics with adjustable parameters.",
            waveform_name
        ));

        ui.add_space(12.0);
        ui.heading("General Signal Processing Pipeline");
        ui.add_space(8.0);

        ui.code(
            r#"
TRANSMIT:
  Data ──► Symbol Mapping ──► Pulse Shaping ──► Upconversion ──► I/Q Samples
              │                   │                 │
           Bits to           Filter for          Mix with
           symbols           bandwidth           carrier

RECEIVE:
  I/Q ──► Downconversion ──► Matched Filter ──► Symbol Detect ──► Data
              │                   │                  │
           Remove            Correlate          Threshold/
           carrier           with TX pulse      decision
"#,
        );

        ui.add_space(12.0);
        ui.label(
            "Select a specific waveform from the sidebar to see detailed information \
            about its modulation characteristics and applications.",
        );
    }
}
