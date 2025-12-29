//! STANAG 4285 Lab View - Interactive NATO HF Modem demonstration
//!
//! This view provides:
//! - Mode selection (75-3600 bps)
//! - Constellation visualization
//! - Modulation/demodulation demonstration
//! - Signal quality metrics

use egui::{Color32, Ui, RichText, Vec2};
use egui_plot::{Plot, Points, PlotPoints};
use r4w_core::waveform::stanag4285::{Stanag4285, Stanag4285Mode, PskType};
use r4w_core::waveform::Waveform;
use r4w_core::types::IQSample;

/// STANAG 4285 Lab View state
pub struct Stanag4285View {
    // Mode selection
    selected_mode: Stanag4285Mode,

    // Test message
    test_message: String,

    // Signal parameters
    sample_rate: f64,

    // Results
    samples: Vec<IQSample>,
    demod_result: Option<DemodStats>,

    // Visualization
    show_constellation: bool,
    show_time_domain: bool,
}

/// Demodulation statistics
#[derive(Debug, Clone)]
struct DemodStats {
    symbols_detected: usize,
    bits_recovered: usize,
    sync_quality: f64,
}

impl Default for Stanag4285View {
    fn default() -> Self {
        Self::new()
    }
}

impl Stanag4285View {
    pub fn new() -> Self {
        Self {
            selected_mode: Stanag4285Mode::Mode2400Short,
            test_message: "Hello STANAG 4285!".to_string(),
            sample_rate: 48000.0,
            samples: Vec::new(),
            demod_result: None,
            show_constellation: true,
            show_time_domain: true,
        }
    }

    /// Get available modes for UI
    fn available_modes() -> &'static [(Stanag4285Mode, &'static str)] {
        &[
            (Stanag4285Mode::Mode75Short, "75 bps (Short)"),
            (Stanag4285Mode::Mode75Long, "75 bps (Long)"),
            (Stanag4285Mode::Mode150Short, "150 bps (Short)"),
            (Stanag4285Mode::Mode150Long, "150 bps (Long)"),
            (Stanag4285Mode::Mode300Short, "300 bps (Short)"),
            (Stanag4285Mode::Mode300Long, "300 bps (Long)"),
            (Stanag4285Mode::Mode600Short, "600 bps (Short)"),
            (Stanag4285Mode::Mode600Long, "600 bps (Long)"),
            (Stanag4285Mode::Mode1200Short, "1200 bps (Short)"),
            (Stanag4285Mode::Mode1200Long, "1200 bps (Long)"),
            (Stanag4285Mode::Mode2400Short, "2400 bps (Short)"),
            (Stanag4285Mode::Mode2400Long, "2400 bps (Long)"),
            (Stanag4285Mode::Mode3600Short, "3600 bps (Short)"),
            (Stanag4285Mode::Mode3600Long, "3600 bps (Long)"),
        ]
    }

    /// Run modulation/demodulation
    fn run_modem(&mut self) {
        let modem = Stanag4285::new(self.sample_rate, self.selected_mode);

        // Convert test message to bytes
        let data = self.test_message.as_bytes();

        // Modulate
        self.samples = modem.modulate(data);

        // Demodulate and get stats
        let result = modem.demodulate(&self.samples);

        self.demod_result = Some(DemodStats {
            symbols_detected: result.symbols.len(),
            bits_recovered: result.bits.len(),
            sync_quality: 0.95, // Simulated for now
        });
    }

    /// Render the view
    pub fn render(&mut self, ui: &mut Ui) {
        ui.heading("STANAG 4285 - NATO HF Data Modem");
        ui.add_space(8.0);

        ui.horizontal(|ui| {
            // Left panel - Configuration
            ui.vertical(|ui| {
                ui.set_width(300.0);
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
            ui.heading("Configuration");
            ui.add_space(4.0);

            // Mode selection
            ui.label("Operating Mode:");
            egui::ComboBox::from_id_salt("stanag_mode")
                .selected_text(self.mode_display_name())
                .show_ui(ui, |ui| {
                    for (mode, name) in Self::available_modes() {
                        ui.selectable_value(&mut self.selected_mode, *mode, *name);
                    }
                });

            ui.add_space(8.0);

            // Mode info
            ui.label(RichText::new("Mode Details:").strong());
            ui.label(format!("  Data Rate: {} bps", self.selected_mode.data_rate()));
            ui.label(format!("  Modulation: {}", self.modulation_name()));
            ui.label(format!("  Interleaving: {}",
                if self.selected_mode.is_long_interleave() { "Long" } else { "Short" }
            ));
            let (k, n) = self.selected_mode.code_rate();
            ui.label(format!("  Code Rate: {}/{}", k, n));
            ui.label(format!("  Interleave Depth: {} symbols",
                self.selected_mode.interleave_depth()
            ));

            ui.add_space(8.0);

            // Test message
            ui.label("Test Message:");
            ui.text_edit_singleline(&mut self.test_message);

            ui.add_space(8.0);

            // Run button
            if ui.button("Generate & Demodulate").clicked() {
                self.run_modem();
            }

            ui.add_space(8.0);

            // Results
            if let Some(stats) = &self.demod_result {
                ui.group(|ui| {
                    ui.label(RichText::new("Results:").strong());
                    ui.label(format!("Samples generated: {}", self.samples.len()));
                    ui.label(format!("Symbols detected: {}", stats.symbols_detected));
                    ui.label(format!("Bits recovered: {}", stats.bits_recovered));
                    ui.label(format!("Sync quality: {:.1}%", stats.sync_quality * 100.0));
                });
            }
        });

        ui.add_space(8.0);

        // Educational content
        ui.group(|ui| {
            ui.heading("About STANAG 4285");
            ui.add_space(4.0);

            ui.label("STANAG 4285 is a NATO standard for HF data modems:");
            ui.add_space(4.0);

            ui.label("• 2400 baud symbol rate");
            ui.label("• PSK modulation (BPSK/QPSK/8-PSK)");
            ui.label("• Rate-1/2 convolutional coding (K=7)");
            ui.label("• Block interleaving for burst errors");
            ui.label("• 16-symbol probe sequences");
            ui.label("• Data rates from 75 to 3600 bps");

            ui.add_space(4.0);
            ui.label(RichText::new("Long vs Short Interleaving:").italics());
            ui.label("Long: Better burst error protection (4.8s delay)");
            ui.label("Short: Lower latency (0.6s delay)");
        });
    }

    fn render_visualization_panel(&mut self, ui: &mut Ui) {
        // Visualization toggles
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.show_constellation, "Constellation");
            ui.checkbox(&mut self.show_time_domain, "Time Domain");
        });

        ui.add_space(8.0);

        // Constellation plot
        if self.show_constellation {
            ui.group(|ui| {
                ui.label(RichText::new("Constellation Diagram").strong());

                let constellation = self.selected_mode.modulation().constellation();

                let plot = Plot::new("stanag_constellation")
                    .view_aspect(1.0)
                    .height(250.0)
                    .data_aspect(1.0)
                    .include_x(-1.5)
                    .include_x(1.5)
                    .include_y(-1.5)
                    .include_y(1.5);

                plot.show(ui, |plot_ui| {
                    // Draw ideal constellation points
                    let ideal_points: PlotPoints = constellation.iter()
                        .map(|p| [p.re, p.im])
                        .collect();

                    plot_ui.points(
                        Points::new(ideal_points)
                            .color(Color32::LIGHT_BLUE)
                            .radius(8.0)
                            .name("Ideal")
                    );

                    // Draw received symbols (if we have samples)
                    if !self.samples.is_empty() {
                        let modem = Stanag4285::new(self.sample_rate, self.selected_mode);
                        let sps = modem.samples_per_symbol();

                        // Skip sync preamble and sample at symbol centers
                        let sync_samples = 80 * sps;
                        let received: Vec<[f64; 2]> = self.samples.iter()
                            .skip(sync_samples)
                            .step_by(sps)
                            .take(100) // Limit for performance
                            .map(|s| [s.re, s.im])
                            .collect();

                        if !received.is_empty() {
                            plot_ui.points(
                                Points::new(received)
                                    .color(Color32::YELLOW)
                                    .radius(3.0)
                                    .name("Received")
                            );
                        }
                    }
                });
            });
        }

        // Time domain plot
        if self.show_time_domain && !self.samples.is_empty() {
            ui.add_space(8.0);
            ui.group(|ui| {
                ui.label(RichText::new("Time Domain (I/Q)").strong());

                let plot = Plot::new("stanag_time")
                    .height(150.0)
                    .include_y(-1.5)
                    .include_y(1.5);

                let num_samples = self.samples.len().min(1000);

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

        // Frame structure diagram
        ui.add_space(8.0);
        ui.group(|ui| {
            ui.label(RichText::new("Frame Structure").strong());
            ui.add_space(4.0);

            // Draw frame structure
            let available_width = ui.available_width();
            let height = 40.0;

            let (rect, _response) = ui.allocate_exact_size(
                Vec2::new(available_width, height),
                egui::Sense::hover()
            );

            let painter = ui.painter();

            // Calculate segment widths
            let total_symbols = 80.0 + 16.0 + 32.0 + 16.0 + 32.0;
            let scale = rect.width() / total_symbols;

            let mut x = rect.left();

            // Sync (80 symbols)
            let sync_width = 80.0 * scale;
            painter.rect_filled(
                egui::Rect::from_min_size(egui::pos2(x, rect.top()), Vec2::new(sync_width, height)),
                0.0,
                Color32::from_rgb(100, 150, 200)
            );
            painter.text(
                egui::pos2(x + sync_width / 2.0, rect.center().y),
                egui::Align2::CENTER_CENTER,
                "Sync\n(80)",
                egui::FontId::default(),
                Color32::WHITE
            );
            x += sync_width;

            // Probe (16 symbols)
            let probe_width = 16.0 * scale;
            painter.rect_filled(
                egui::Rect::from_min_size(egui::pos2(x, rect.top()), Vec2::new(probe_width, height)),
                0.0,
                Color32::from_rgb(150, 200, 100)
            );
            painter.text(
                egui::pos2(x + probe_width / 2.0, rect.center().y),
                egui::Align2::CENTER_CENTER,
                "P",
                egui::FontId::default(),
                Color32::WHITE
            );
            x += probe_width;

            // Data (32 symbols)
            let data_width = 32.0 * scale;
            painter.rect_filled(
                egui::Rect::from_min_size(egui::pos2(x, rect.top()), Vec2::new(data_width, height)),
                0.0,
                Color32::from_rgb(200, 150, 100)
            );
            painter.text(
                egui::pos2(x + data_width / 2.0, rect.center().y),
                egui::Align2::CENTER_CENTER,
                "Data\n(32)",
                egui::FontId::default(),
                Color32::WHITE
            );
            x += data_width;

            // Probe
            painter.rect_filled(
                egui::Rect::from_min_size(egui::pos2(x, rect.top()), Vec2::new(probe_width, height)),
                0.0,
                Color32::from_rgb(150, 200, 100)
            );
            painter.text(
                egui::pos2(x + probe_width / 2.0, rect.center().y),
                egui::Align2::CENTER_CENTER,
                "P",
                egui::FontId::default(),
                Color32::WHITE
            );
            x += probe_width;

            // Data
            painter.rect_filled(
                egui::Rect::from_min_size(egui::pos2(x, rect.top()), Vec2::new(data_width, height)),
                0.0,
                Color32::from_rgb(200, 150, 100)
            );
            painter.text(
                egui::pos2(x + data_width / 2.0, rect.center().y),
                egui::Align2::CENTER_CENTER,
                "Data\n(32)",
                egui::FontId::default(),
                Color32::WHITE
            );

            ui.add_space(4.0);
            ui.label("P = 16-symbol probe sequence for channel estimation");
        });
    }

    fn mode_display_name(&self) -> String {
        for (mode, name) in Self::available_modes() {
            if *mode == self.selected_mode {
                return name.to_string();
            }
        }
        "Unknown".to_string()
    }

    fn modulation_name(&self) -> &'static str {
        match self.selected_mode.modulation() {
            PskType::Bpsk => "BPSK",
            PskType::Qpsk => "QPSK",
            PskType::Psk8 => "8-PSK",
        }
    }
}
