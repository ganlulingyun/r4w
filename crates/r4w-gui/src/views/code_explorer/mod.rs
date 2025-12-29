//! Code Explorer View
//!
//! An educational view that displays the actual Rust implementation
//! of waveform processing with syntax highlighting and explanations.

pub mod am_snippets;
pub mod css_snippets;
pub mod cw_snippets;
pub mod dsss_snippets;
pub mod fhss_snippets;
pub mod fm_snippets;
pub mod fmcw_snippets;
pub mod fsk_snippets;
pub mod highlight;
pub mod ofdm_snippets;
pub mod ook_snippets;
pub mod ppm_snippets;
pub mod psk_snippets;
pub mod qam_snippets;
pub mod snippets;
pub mod uwb_snippets;
pub mod zigbee_snippets;

use eframe::egui::{self, Color32, RichText, ScrollArea, Ui};
use snippets::{CodeCategory, CodeRegistry, CodeSnippet, WaveformCode};
use std::collections::HashSet;

/// Map sidebar waveform names to Code Explorer waveform IDs
fn map_waveform_to_code_id(waveform: &str) -> &'static str {
    match waveform {
        "CW" => "CW",
        "OOK" => "OOK",
        "AM" => "AM",
        "FM" => "FM",
        "PPM" | "ADS-B" => "PPM",
        "BFSK" | "4-FSK" => "FSK",
        "BPSK" | "QPSK" | "8-PSK" => "PSK",
        "16-QAM" | "64-QAM" | "256-QAM" => "QAM",
        "OFDM" => "OFDM",
        "LoRa" => "CSS",
        "DSSS" | "DSSS-QPSK" => "DSSS",
        "FHSS" => "FHSS",
        "Zigbee" => "Zigbee",
        "UWB" => "UWB",
        "FMCW" => "FMCW",
        _ => "CW", // Default fallback
    }
}

/// Code Explorer view state
pub struct CodeExplorerView {
    /// Currently selected waveform ID (mapped from sidebar selection)
    selected_waveform: String,
    /// Set of expanded category names
    expanded_categories: HashSet<String>,
    /// Set of expanded snippet names (format: "category::snippet")
    expanded_snippets: HashSet<String>,
}

impl Default for CodeExplorerView {
    fn default() -> Self {
        Self::new()
    }
}

impl CodeExplorerView {
    /// Create a new Code Explorer view
    pub fn new() -> Self {
        let mut expanded_categories = HashSet::new();
        // Start with first category expanded
        expanded_categories.insert("Signal Generation".to_string());

        Self {
            selected_waveform: "CW".to_string(),
            expanded_categories,
            expanded_snippets: HashSet::new(),
        }
    }

    /// Set the waveform from sidebar selection (maps to Code Explorer ID)
    pub fn set_waveform(&mut self, sidebar_waveform: &str) {
        let code_id = map_waveform_to_code_id(sidebar_waveform);
        if self.selected_waveform != code_id {
            self.selected_waveform = code_id.to_string();
            // Reset expanded state when switching waveforms
            self.expanded_categories.clear();
            self.expanded_snippets.clear();
            // Expand first category
            if let Some(waveform) = CodeRegistry::get_waveform(&self.selected_waveform) {
                if let Some(first) = waveform.categories.first() {
                    self.expanded_categories.insert(first.name.to_string());
                }
            }
        }
    }

    /// Render the Code Explorer view
    pub fn render(&mut self, ui: &mut Ui) {
        self.render_with_waveform(ui, None)
    }

    /// Render with optional external waveform selection
    pub fn render_with_waveform(&mut self, ui: &mut Ui, sidebar_waveform: Option<&str>) {
        // Sync with sidebar selection if provided
        if let Some(wf) = sidebar_waveform {
            self.set_waveform(wf);
        }

        ScrollArea::vertical().show(ui, |ui| {
            self.render_header(ui);
            ui.add_space(10.0);

            // Show current waveform info (no dropdown - controlled by sidebar)
            self.render_waveform_info(ui);
            ui.add_space(15.0);

            if let Some(waveform) = CodeRegistry::get_waveform(&self.selected_waveform) {
                self.render_waveform_intro(ui, waveform);
                ui.add_space(15.0);

                for category in waveform.categories {
                    self.render_category(ui, category);
                    ui.add_space(10.0);
                }
            }

            ui.add_space(20.0);
            self.render_footer(ui);
        });
    }

    /// Render the header section
    fn render_header(&self, ui: &mut Ui) {
        ui.heading("Code Explorer");
        ui.add_space(5.0);
        ui.label(
            RichText::new(
                "Explore the actual Rust implementation of waveform processing. \
                Each function is shown with syntax highlighting and educational explanations.",
            )
            .weak(),
        );
    }

    /// Render waveform info (controlled by sidebar, no dropdown)
    fn render_waveform_info(&self, ui: &mut Ui) {
        if let Some(waveform) = CodeRegistry::get_waveform(&self.selected_waveform) {
            ui.horizontal(|ui| {
                ui.label("Viewing:");
                ui.label(RichText::new(waveform.display_name).strong());

                ui.add_space(10.0);
                let stars = "★".repeat(waveform.complexity as usize);
                let empty = "☆".repeat(5 - waveform.complexity as usize);
                ui.label(
                    RichText::new(format!("Complexity: {}{}", stars, empty))
                        .color(Color32::from_rgb(255, 200, 100)),
                );
            });
            ui.label(
                RichText::new("Select a waveform from the sidebar to view its implementation.")
                    .weak()
                    .italics(),
            );
        }
    }

    /// Render the waveform introduction
    fn render_waveform_intro(&self, ui: &mut Ui, waveform: &WaveformCode) {
        egui::Frame::none()
            .fill(Color32::from_rgb(40, 45, 55))
            .inner_margin(12.0)
            .rounding(6.0)
            .show(ui, |ui| {
                ui.label(RichText::new(waveform.display_name).strong().size(16.0));
                ui.add_space(5.0);
                ui.label(waveform.introduction);
            });
    }

    /// Render a category of code snippets
    fn render_category(&mut self, ui: &mut Ui, category: &CodeCategory) {
        let is_expanded = self.expanded_categories.contains(category.name);

        let header = egui::CollapsingHeader::new(
            RichText::new(category.name).strong().size(14.0),
        )
        .default_open(is_expanded)
        .show(ui, |ui| {
            ui.label(RichText::new(category.description).weak().italics());
            ui.add_space(10.0);

            for snippet in category.snippets {
                self.render_snippet(ui, category.name, snippet);
                ui.add_space(8.0);
            }
        });

        // Track expansion state
        if header.header_response.clicked() {
            if is_expanded {
                self.expanded_categories.remove(category.name);
            } else {
                self.expanded_categories.insert(category.name.to_string());
            }
        }
    }

    /// Render a single code snippet
    fn render_snippet(&mut self, ui: &mut Ui, category_name: &str, snippet: &CodeSnippet) {
        let snippet_key = format!("{}::{}", category_name, snippet.name);
        let is_expanded = self.expanded_snippets.contains(&snippet_key);

        // Snippet header
        egui::Frame::none()
            .fill(Color32::from_rgb(35, 40, 48))
            .inner_margin(8.0)
            .rounding(4.0)
            .show(ui, |ui| {
                // Function name and brief description
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(snippet.name)
                            .monospace()
                            .color(Color32::from_rgb(220, 220, 170)),
                    );
                    ui.label(RichText::new("—").weak());
                    ui.label(RichText::new(snippet.brief).weak());
                });

                ui.add_space(8.0);

                // Code block with dark background
                egui::Frame::none()
                    .fill(Color32::from_rgb(25, 28, 32))
                    .inner_margin(10.0)
                    .rounding(4.0)
                    .show(ui, |ui| {
                        let highlighted = highlight::highlight_rust_code(snippet.code);
                        ui.add(egui::Label::new(highlighted).wrap_mode(egui::TextWrapMode::Extend));
                    });

                ui.add_space(8.0);

                // Expand/collapse button for explanation
                let expand_text = if is_expanded {
                    "▼ Hide Explanation"
                } else {
                    "▶ Show Explanation"
                };

                if ui
                    .button(RichText::new(expand_text).small())
                    .clicked()
                {
                    if is_expanded {
                        self.expanded_snippets.remove(&snippet_key);
                    } else {
                        self.expanded_snippets.insert(snippet_key.clone());
                    }
                }

                // Explanation (if expanded)
                if is_expanded {
                    ui.add_space(8.0);

                    egui::Frame::none()
                        .fill(Color32::from_rgb(45, 50, 60))
                        .inner_margin(10.0)
                        .rounding(4.0)
                        .show(ui, |ui| {
                            // Render explanation with basic markdown-like formatting
                            self.render_explanation(ui, snippet.explanation);

                            // Key concepts
                            if !snippet.concepts.is_empty() {
                                ui.add_space(10.0);
                                ui.horizontal_wrapped(|ui| {
                                    ui.label(RichText::new("Key concepts:").strong());
                                    for concept in snippet.concepts {
                                        egui::Frame::none()
                                            .fill(Color32::from_rgb(70, 80, 100))
                                            .inner_margin(egui::vec2(6.0, 2.0))
                                            .rounding(3.0)
                                            .show(ui, |ui| {
                                                ui.label(
                                                    RichText::new(*concept)
                                                        .small()
                                                        .color(Color32::WHITE),
                                                );
                                            });
                                    }
                                });
                            }
                        });
                }
            });
    }

    /// Render explanation text with basic formatting
    fn render_explanation(&self, ui: &mut Ui, text: &str) {
        for line in text.lines() {
            if line.starts_with("**") && line.ends_with("**") {
                // Bold heading
                let content = line.trim_start_matches("**").trim_end_matches("**");
                ui.label(RichText::new(content).strong());
            } else if line.starts_with("**") {
                // Line starting with bold
                let parts: Vec<&str> = line.splitn(3, "**").collect();
                if parts.len() >= 3 {
                    ui.horizontal_wrapped(|ui| {
                        ui.label(RichText::new(parts[1]).strong());
                        ui.label(parts[2]);
                    });
                } else {
                    ui.label(line);
                }
            } else if line.starts_with("- ") {
                // Bullet point
                ui.horizontal(|ui| {
                    ui.label("  •");
                    ui.label(&line[2..]);
                });
            } else if line.is_empty() {
                ui.add_space(5.0);
            } else {
                ui.label(line);
            }
        }
    }

    /// Get learning tips specific to the currently selected waveform
    fn get_learning_tips(&self) -> (&'static str, &'static str, Vec<&'static str>) {
        match self.selected_waveform.as_str() {
            "CW" => (
                "Continuous Wave (CW)",
                "The foundation of all RF signals",
                vec![
                    "CW is the simplest waveform: a pure sinusoid at a single frequency",
                    "I/Q representation: I = cos(2πft), Q = sin(2πft) - this is Euler's formula in action",
                    "Phase accumulator technique prevents discontinuities when generating signals",
                    "Used as carrier wave in all other modulation schemes",
                    "Try changing the frequency and observe how I and Q traces change",
                ],
            ),
            "OOK" => (
                "On-Off Keying (OOK)",
                "The simplest digital modulation",
                vec![
                    "OOK encodes bits by turning the carrier on (1) or off (0)",
                    "Bandwidth efficiency: 1 bit per symbol, but simple to implement",
                    "Used in early radio, garage door openers, and RFID systems",
                    "Susceptible to noise since 'off' state has no energy to detect",
                    "Watch the constellation: points at origin (off) and on the real axis (on)",
                ],
            ),
            "AM" => (
                "Amplitude Modulation (AM)",
                "Classic analog modulation",
                vec![
                    "AM varies the carrier amplitude to encode the message signal",
                    "Envelope detection is simple: just rectify and filter",
                    "Modulation index m = (A_max - A_min) / (A_max + A_min)",
                    "m > 1 causes overmodulation and distortion",
                    "AM radio uses DSB-FC (Double Sideband Full Carrier) for simple receivers",
                    "Observe how the envelope follows your modulating signal shape",
                ],
            ),
            "FM" => (
                "Frequency Modulation (FM)",
                "Noise-resistant analog modulation",
                vec![
                    "FM encodes information in frequency variations, not amplitude",
                    "Constant envelope makes FM resistant to amplitude noise and fading",
                    "Carson's rule: BW ≈ 2(Δf + f_m) where Δf is deviation, f_m is max modulating freq",
                    "Deviation index β = Δf / f_m determines narrowband (β<1) vs wideband (β>1)",
                    "FM capture effect: stronger signal 'captures' the receiver, rejecting weaker",
                    "Watch the spectrogram to see frequency changes over time",
                ],
            ),
            "PPM" => (
                "Pulse Position Modulation (PPM)",
                "Time-domain digital encoding",
                vec![
                    "PPM encodes data in the timing (position) of pulses within a frame",
                    "ADS-B uses PPM: each bit is a 1μs pulse in one of two positions per bit period",
                    "Excellent noise immunity since only pulse presence/absence matters",
                    "Synchronization is critical - receiver must know frame boundaries",
                    "Used in optical communication, IR remotes, and aircraft transponders",
                    "Observe how bit values determine whether pulse is early or late in each slot",
                ],
            ),
            "FSK" => (
                "Frequency Shift Keying (FSK)",
                "Robust digital modulation",
                vec![
                    "FSK maps digital symbols to discrete frequencies",
                    "BFSK uses 2 frequencies, 4-FSK uses 4 (encoding 2 bits per symbol)",
                    "Phase continuity (CPFSK) prevents spectral splatter at transitions",
                    "Deviation index h = Δf × T_symbol determines spectral efficiency",
                    "MSK (h=0.5) is bandwidth-optimal continuous-phase FSK",
                    "Demodulation: compare energy at each frequency bin",
                    "Watch how the instantaneous frequency jumps between discrete levels",
                ],
            ),
            "PSK" => (
                "Phase Shift Keying (PSK)",
                "Bandwidth-efficient modulation",
                vec![
                    "PSK encodes data in the phase of the carrier signal",
                    "BPSK: 1 bit/symbol (0°/180°), QPSK: 2 bits/symbol (0°/90°/180°/270°)",
                    "Constellation diagram shows all possible symbol positions",
                    "Gray coding ensures adjacent symbols differ by only 1 bit",
                    "Coherent demodulation requires carrier phase recovery (PLL, Costas loop)",
                    "QPSK has same BER as BPSK but double the data rate!",
                    "Watch constellation points - each point is a unique symbol",
                ],
            ),
            "QAM" => (
                "Quadrature Amplitude Modulation (QAM)",
                "Maximum spectral efficiency",
                vec![
                    "QAM combines amplitude AND phase to pack more bits per symbol",
                    "16-QAM: 4 bits/symbol, 64-QAM: 6 bits/symbol, 256-QAM: 8 bits/symbol",
                    "Higher orders need better SNR: each doubling needs ~3dB more",
                    "Used in WiFi, cable modems, 4G/5G cellular networks",
                    "Constellation shows a grid of points in the I-Q plane",
                    "Minimum distance between points determines noise tolerance",
                    "Gray coding maps adjacent points to symbols differing by 1 bit",
                ],
            ),
            "OFDM" => (
                "Orthogonal Frequency Division Multiplexing",
                "Modern multi-carrier modulation",
                vec![
                    "OFDM splits data across many narrowband orthogonal subcarriers",
                    "Each subcarrier uses QAM/PSK - parallelism enables high data rates",
                    "FFT/IFFT efficiently implements modulation/demodulation",
                    "Cyclic prefix eliminates inter-symbol interference from multipath",
                    "Subcarrier spacing = 1/symbol_duration ensures orthogonality",
                    "Used in WiFi, LTE, 5G, DVB-T, and DAB radio",
                    "Watch the frequency domain - each spike is a subcarrier",
                ],
            ),
            "CSS" => (
                "Chirp Spread Spectrum (LoRa)",
                "Long-range IoT modulation",
                vec![
                    "CSS encodes data by cyclically shifting chirps in time/frequency",
                    "Spreading Factor (SF 7-12) trades data rate for range and noise immunity",
                    "Each SF doubles the range but halves the data rate",
                    "Demodulation: multiply by downchirp, FFT, find peak position = symbol",
                    "Processing gain = 2^SF gives exceptional sensitivity (-137 dBm at SF12)",
                    "Resistant to multipath, Doppler, and interference",
                    "Watch the spectrogram: upward-sweeping lines are the chirps",
                ],
            ),
            "DSSS" => (
                "Direct Sequence Spread Spectrum (DSSS)",
                "Processing gain through PN spreading",
                vec![
                    "DSSS multiplies each data symbol by a pseudo-noise (PN) sequence",
                    "Processing gain = 10×log₁₀(chips_per_symbol) dB",
                    "Signal can operate below the noise floor with enough gain",
                    "Used in GPS (30 dB gain), CDMA cellular, and military comms",
                    "Different PN codes enable Code Division Multiple Access (CDMA)",
                    "Gold codes provide good cross-correlation for multi-user systems",
                    "Receiver correlates with local PN replica to despread",
                ],
            ),
            "FHSS" => (
                "Frequency Hopping Spread Spectrum (FHSS)",
                "Anti-jamming through frequency agility",
                vec![
                    "FHSS rapidly switches carrier frequency per pseudo-random pattern",
                    "Jamming resistance: jammer can't follow the hops fast enough",
                    "Used in Bluetooth (79 channels, 1600 hops/sec) and military radios",
                    "Processing gain = 10×log₁₀(num_channels) dB vs narrowband jammer",
                    "TX and RX must be synchronized to hop at same times",
                    "Different hop patterns enable multiple simultaneous users",
                    "Watch the spectrogram: signal jumps between frequency bands",
                ],
            ),
            "Zigbee" => (
                "Zigbee (IEEE 802.15.4)",
                "Low-power IoT with O-QPSK and DSSS",
                vec![
                    "Zigbee uses O-QPSK with half-sine pulse shaping for constant envelope",
                    "DSSS spreading: 4-bit symbols → 32-chip sequences",
                    "Data rate: 250 kbps at 2.4 GHz (2 Mchip/s)",
                    "O-QPSK offsets I and Q to avoid 180° phase transitions",
                    "Half-sine shaping creates MSK-like spectral efficiency",
                    "Used in smart home, industrial sensors, and mesh networks",
                    "Battery life of years due to low power and low data rate",
                ],
            ),
            "UWB" => (
                "Ultra-Wideband (UWB)",
                "Precision ranging with nanosecond pulses",
                vec![
                    "UWB uses extremely short pulses (1-2 ns) for very wide bandwidth",
                    "Range resolution: ~15 cm per nanosecond of pulse width",
                    "FCC allows operation below noise floor (-41.3 dBm/MHz)",
                    "Used in Apple AirTags, secure car keys, and indoor positioning",
                    "Pulse shapes: Gaussian monocycle, doublet (DC-free)",
                    "Can resolve multipath (separate direct from reflected paths)",
                    "Time-of-flight ranging achieves cm-level accuracy",
                ],
            ),
            "FMCW" => (
                "FMCW Radar",
                "Range and velocity from frequency-swept chirps",
                vec![
                    "FMCW transmits continuous frequency-swept chirps (not pulses)",
                    "Range from beat frequency: R = f_beat × c × T / (2 × BW)",
                    "Velocity from Doppler FFT across multiple chirps",
                    "Used in automotive radar (77 GHz), altimeters, level sensors",
                    "Range resolution = c / (2 × BW), e.g., 4 GHz → 3.75 cm",
                    "2D-FFT processing creates range-Doppler maps",
                    "Dechirping converts wideband chirp to narrowband beat tone",
                ],
            ),
            _ => (
                "Unknown Waveform",
                "Select a waveform from the sidebar",
                vec![
                    "This waveform doesn't have specific learning tips yet",
                    "Try exploring the code snippets above",
                    "Each waveform demonstrates different DSP concepts",
                ],
            ),
        }
    }

    /// Render the footer with waveform-specific learning tips
    fn render_footer(&self, ui: &mut Ui) {
        ui.separator();
        ui.add_space(5.0);

        let (waveform_name, subtitle, tips) = self.get_learning_tips();

        egui::Frame::none()
            .fill(Color32::from_rgb(35, 50, 45))
            .inner_margin(12.0)
            .rounding(6.0)
            .show(ui, |ui| {
                // Header with waveform name
                ui.horizontal(|ui| {
                    ui.label(RichText::new("Learning Tips:").strong().size(14.0));
                    ui.label(
                        RichText::new(waveform_name)
                            .strong()
                            .size(14.0)
                            .color(Color32::from_rgb(130, 200, 160)),
                    );
                });
                ui.label(RichText::new(subtitle).weak().italics());
                ui.add_space(8.0);

                // Tips as bullet points
                for tip in tips {
                    ui.horizontal_wrapped(|ui| {
                        ui.label(RichText::new("•").strong().color(Color32::from_rgb(100, 180, 140)));
                        ui.label(tip);
                    });
                }

                ui.add_space(8.0);
                ui.separator();
                ui.add_space(4.0);

                // General tip
                ui.label(
                    RichText::new("Switch waveforms in the sidebar to see different implementations and tips")
                        .weak()
                        .italics()
                        .small(),
                );
            });
    }
}
