//! ALE Lab View - Interactive Automatic Link Establishment demonstration
//!
//! This view provides:
//! - Station address configuration
//! - Call generation and visualization
//! - Tone spectrum display
//! - Link Quality Analysis (LQA)

use egui::{Color32, Ui, RichText, Vec2};
use egui_plot::{Plot, PlotPoints, Bar, BarChart};
use r4w_core::waveform::ale::{Ale, AleLqa};
use r4w_core::types::IQSample;

/// ALE tone frequencies
const ALE_TONES: [f64; 8] = [750.0, 1000.0, 1250.0, 1500.0, 1750.0, 2000.0, 2250.0, 2500.0];

/// ALE Lab View state
pub struct AleView {
    // Station configuration
    my_address: String,
    target_address: String,

    // Signal parameters
    sample_rate: f64,

    // Call parameters
    word_repeats: usize,

    // Results
    samples: Vec<IQSample>,
    detected_words: Vec<DetectedWord>,
    lqa: Option<AleLqa>,

    // Visualization
    show_spectrum: bool,
    show_tones: bool,
    show_protocol: bool,
}

/// Detected word with metadata
#[derive(Debug, Clone)]
struct DetectedWord {
    word_type: String,
    data: String,
}

impl Default for AleView {
    fn default() -> Self {
        Self::new()
    }
}

impl AleView {
    pub fn new() -> Self {
        Self {
            my_address: "HOME".to_string(),
            target_address: "REMOTE".to_string(),
            sample_rate: 48000.0,
            word_repeats: 2,
            samples: Vec::new(),
            detected_words: Vec::new(),
            lqa: None,
            show_spectrum: true,
            show_tones: true,
            show_protocol: true,
        }
    }

    /// Generate a call and analyze it
    fn generate_call(&mut self) {
        let mut ale = Ale::new(self.sample_rate, &self.my_address);

        // Generate call sequence
        self.samples = ale.generate_call(&self.target_address, self.word_repeats);

        // Demodulate and detect words
        let words = ale.demodulate_words(&self.samples);
        self.detected_words = words.iter()
            .map(|w| DetectedWord {
                word_type: format!("{:?}", w.word_type),
                data: format!("0x{:06X}", w.data),
            })
            .collect();

        // Calculate LQA
        self.lqa = Some(ale.calculate_lqa(&self.samples));
    }

    /// Render the view
    pub fn render(&mut self, ui: &mut Ui) {
        ui.heading("ALE - Automatic Link Establishment");
        ui.add_space(8.0);

        ui.horizontal(|ui| {
            // Left panel - Configuration
            ui.vertical(|ui| {
                ui.set_width(320.0);
                self.render_config_panel(ui);
            });

            ui.separator();

            // Right panel - Visualization
            ui.vertical(|ui| {
                self.render_visualization_panel(ui);
            });
        });
    }

    fn render_config_panel(&mut self, ui: &mut Ui) {
        ui.group(|ui| {
            ui.heading("Station Configuration");
            ui.add_space(4.0);

            ui.horizontal(|ui| {
                ui.label("My Address:");
                ui.text_edit_singleline(&mut self.my_address);
            });

            ui.horizontal(|ui| {
                ui.label("Target Address:");
                ui.text_edit_singleline(&mut self.target_address);
            });

            ui.add_space(4.0);

            ui.horizontal(|ui| {
                ui.label("Word Repeats:");
                ui.add(egui::Slider::new(&mut self.word_repeats, 1..=5));
            });

            ui.add_space(8.0);

            if ui.button("Generate ALE Call").clicked() {
                self.generate_call();
            }
        });

        ui.add_space(8.0);

        // Results
        if !self.detected_words.is_empty() {
            ui.group(|ui| {
                ui.label(RichText::new("Detected Words:").strong());
                ui.add_space(4.0);

                egui::ScrollArea::vertical()
                    .max_height(150.0)
                    .show(ui, |ui| {
                        for (i, word) in self.detected_words.iter().enumerate() {
                            ui.horizontal(|ui| {
                                ui.label(format!("{}: ", i + 1));
                                ui.label(RichText::new(&word.word_type).color(Color32::LIGHT_BLUE));
                                ui.label(&word.data);
                            });
                        }
                    });
            });
        }

        // LQA Results
        if let Some(lqa) = &self.lqa {
            ui.add_space(8.0);
            ui.group(|ui| {
                ui.label(RichText::new("Link Quality Analysis:").strong());
                ui.add_space(4.0);

                ui.horizontal(|ui| {
                    ui.label("Overall Score:");
                    let score = lqa.score();
                    let color = if score > 70 {
                        Color32::GREEN
                    } else if score > 40 {
                        Color32::YELLOW
                    } else {
                        Color32::RED
                    };
                    ui.label(RichText::new(format!("{}%", score)).color(color).strong());
                });

                ui.label(format!("BER Estimate: {}%", lqa.ber));
                ui.label(format!("SINAD: {}%", lqa.sinad));
                ui.label(format!("Multipath: {}%", lqa.multipath));
            });
        }

        ui.add_space(8.0);

        // Educational content
        ui.group(|ui| {
            ui.heading("About ALE");
            ui.add_space(4.0);

            ui.label("ALE (MIL-STD-188-141) provides:");
            ui.add_space(4.0);

            ui.label("• Automatic link establishment");
            ui.label("• Best frequency selection");
            ui.label("• Link quality analysis");
            ui.label("• Station addressing (15 chars)");

            ui.add_space(4.0);
            ui.label(RichText::new("Signal Characteristics:").strong());
            ui.label("• 8-FSK modulation (8 tones)");
            ui.label("• 125 baud symbol rate");
            ui.label("• Golay(24,12) FEC");
            ui.label("• 24-bit words (8 tribits)");
        });

        ui.add_space(8.0);

        // Word types reference
        ui.group(|ui| {
            ui.label(RichText::new("ALE Word Types:").strong());
            ui.add_space(4.0);

            let word_types = [
                ("TO", "001", "Called station address"),
                ("TIS", "011", "Calling station (This Is)"),
                ("TWAS", "010", "Third-party station"),
                ("DATA", "101", "Link data"),
                ("REP", "110", "Repeat request"),
                ("CMD", "111", "Command word"),
            ];

            for (name, preamble, desc) in word_types {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(name).color(Color32::LIGHT_BLUE).monospace());
                    ui.label(format!("({}) - {}", preamble, desc));
                });
            }
        });
    }

    fn render_visualization_panel(&mut self, ui: &mut Ui) {
        // Visualization toggles
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.show_spectrum, "Tone Spectrum");
            ui.checkbox(&mut self.show_tones, "Tone Diagram");
            ui.checkbox(&mut self.show_protocol, "Protocol");
        });

        ui.add_space(8.0);

        // Tone spectrum (bar chart)
        if self.show_tones {
            ui.group(|ui| {
                ui.label(RichText::new("8-FSK Tone Frequencies").strong());
                ui.add_space(4.0);

                let plot = Plot::new("ale_tones")
                    .height(150.0)
                    .include_y(0.0)
                    .include_y(3000.0)
                    .show_axes([false, true]);

                plot.show(ui, |plot_ui| {
                    let bars: Vec<Bar> = ALE_TONES.iter()
                        .enumerate()
                        .map(|(i, &freq)| {
                            Bar::new(i as f64, freq)
                                .width(0.8)
                                .name(format!("{}: {} Hz", i, freq as i32))
                        })
                        .collect();

                    plot_ui.bar_chart(
                        BarChart::new(bars)
                            .color(Color32::LIGHT_BLUE)
                            .name("Tone Frequencies")
                    );
                });

                ui.add_space(4.0);
                ui.label("Tribit values: 000=750Hz, 001=1000Hz, ..., 111=2500Hz");
            });
        }

        // Time domain / spectrum visualization
        if self.show_spectrum && !self.samples.is_empty() {
            ui.add_space(8.0);
            ui.group(|ui| {
                ui.label(RichText::new("Signal Waveform").strong());

                let plot = Plot::new("ale_signal")
                    .height(150.0)
                    .include_y(-1.5)
                    .include_y(1.5);

                let num_samples = self.samples.len().min(2000);

                plot.show(ui, |plot_ui| {
                    // I component
                    let i_line: PlotPoints = self.samples.iter()
                        .take(num_samples)
                        .enumerate()
                        .map(|(i, s)| [i as f64, s.re])
                        .collect();

                    plot_ui.line(
                        egui_plot::Line::new(i_line)
                            .color(Color32::LIGHT_BLUE)
                            .name("I")
                    );

                    // Q component
                    let q_line: PlotPoints = self.samples.iter()
                        .take(num_samples)
                        .enumerate()
                        .map(|(i, s)| [i as f64, s.im])
                        .collect();

                    plot_ui.line(
                        egui_plot::Line::new(q_line)
                            .color(Color32::LIGHT_RED)
                            .name("Q")
                    );
                });
            });
        }

        // Protocol diagram
        if self.show_protocol {
            ui.add_space(8.0);
            ui.group(|ui| {
                ui.label(RichText::new("ALE Call Protocol").strong());
                ui.add_space(4.0);

                // Draw protocol sequence
                let available_width = ui.available_width();
                let height = 80.0;

                let (rect, _response) = ui.allocate_exact_size(
                    Vec2::new(available_width, height),
                    egui::Sense::hover()
                );

                let painter = ui.painter();

                // Background
                painter.rect_filled(rect, 4.0, Color32::from_gray(40));

                // Draw call sequence
                let segment_width = rect.width() / 6.0;
                let segment_height = 30.0;
                let y = rect.top() + 10.0;

                let segments = [
                    ("TO", Color32::from_rgb(100, 150, 200), "Called station"),
                    ("TO", Color32::from_rgb(100, 150, 200), "(repeat)"),
                    ("TIS", Color32::from_rgb(150, 200, 100), "Calling station"),
                    ("TIS", Color32::from_rgb(150, 200, 100), "(repeat)"),
                    ("...", Color32::from_rgb(100, 100, 100), "More words"),
                    ("", Color32::from_rgb(80, 80, 80), ""),
                ];

                for (i, (name, color, desc)) in segments.iter().enumerate() {
                    let x = rect.left() + i as f32 * segment_width;

                    if !name.is_empty() {
                        painter.rect_filled(
                            egui::Rect::from_min_size(
                                egui::pos2(x + 2.0, y),
                                Vec2::new(segment_width - 4.0, segment_height)
                            ),
                            4.0,
                            *color
                        );

                        painter.text(
                            egui::pos2(x + segment_width / 2.0, y + segment_height / 2.0),
                            egui::Align2::CENTER_CENTER,
                            *name,
                            egui::FontId::default(),
                            Color32::WHITE
                        );

                        painter.text(
                            egui::pos2(x + segment_width / 2.0, y + segment_height + 12.0),
                            egui::Align2::CENTER_CENTER,
                            *desc,
                            egui::FontId::proportional(10.0),
                            Color32::GRAY
                        );
                    }
                }

                // Arrow
                let arrow_y = y + segment_height + 30.0;
                painter.arrow(
                    egui::pos2(rect.left() + 20.0, arrow_y),
                    egui::vec2(rect.width() - 40.0, 0.0),
                    egui::Stroke::new(2.0, Color32::GRAY)
                );
                painter.text(
                    egui::pos2(rect.center().x, arrow_y + 10.0),
                    egui::Align2::CENTER_CENTER,
                    "Time →",
                    egui::FontId::proportional(10.0),
                    Color32::GRAY
                );
            });
        }

        // Word structure diagram
        ui.add_space(8.0);
        ui.group(|ui| {
            ui.label(RichText::new("ALE Word Structure (24 bits)").strong());
            ui.add_space(4.0);

            let available_width = ui.available_width();
            let height = 50.0;

            let (rect, _response) = ui.allocate_exact_size(
                Vec2::new(available_width, height),
                egui::Sense::hover()
            );

            let painter = ui.painter();

            // Word is 24 bits: 3-bit preamble + 21-bit data
            let total_bits = 24.0;
            let bit_width = rect.width() / total_bits;

            // Preamble (3 bits)
            let preamble_width = 3.0 * bit_width;
            painter.rect_filled(
                egui::Rect::from_min_size(egui::pos2(rect.left(), rect.top()), Vec2::new(preamble_width, height - 15.0)),
                0.0,
                Color32::from_rgb(100, 150, 200)
            );
            painter.text(
                egui::pos2(rect.left() + preamble_width / 2.0, rect.top() + (height - 15.0) / 2.0),
                egui::Align2::CENTER_CENTER,
                "Pre\n(3)",
                egui::FontId::default(),
                Color32::WHITE
            );

            // Data (21 bits)
            let data_width = 21.0 * bit_width;
            painter.rect_filled(
                egui::Rect::from_min_size(
                    egui::pos2(rect.left() + preamble_width, rect.top()),
                    Vec2::new(data_width, height - 15.0)
                ),
                0.0,
                Color32::from_rgb(200, 150, 100)
            );
            painter.text(
                egui::pos2(rect.left() + preamble_width + data_width / 2.0, rect.top() + (height - 15.0) / 2.0),
                egui::Align2::CENTER_CENTER,
                "Data / Address (21 bits = 3 chars × 6 bits + 3)",
                egui::FontId::default(),
                Color32::WHITE
            );

            ui.add_space(4.0);
            ui.label("Each word transmitted as 8 tribits (3 bits each) = 8 FSK tones");
        });
    }
}
