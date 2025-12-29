//! Chirp signal visualization view

use egui::Ui;
use egui_plot::{Line, Plot, PlotPoints};
use r4w_core::chirp::ChirpGenerator;
use r4w_core::params::LoRaParams;
use r4w_core::types::IQSample;

pub struct ChirpView {
    /// Symbol to display
    selected_symbol: u16,
    /// Show I component
    show_i: bool,
    /// Show Q component
    show_q: bool,
    /// Show frequency
    show_freq: bool,
    /// Zoom level
    zoom: f32,
}

impl ChirpView {
    pub fn new() -> Self {
        Self {
            selected_symbol: 0,
            show_i: true,
            show_q: true,
            show_freq: true,
            zoom: 1.0,
        }
    }

    pub fn render(&mut self, ui: &mut Ui, params: &LoRaParams, _samples: &Option<Vec<IQSample>>) {
        ui.horizontal(|ui| {
            // Controls
            ui.group(|ui| {
                ui.vertical(|ui| {
                    ui.heading("Controls");
                    ui.add_space(8.0);

                    let max_symbol = params.chips_per_symbol() as u16 - 1;
                    ui.add(
                        egui::Slider::new(&mut self.selected_symbol, 0..=max_symbol)
                            .text("Symbol"),
                    );

                    ui.checkbox(&mut self.show_i, "Show I (Real)");
                    ui.checkbox(&mut self.show_q, "Show Q (Imaginary)");
                    ui.checkbox(&mut self.show_freq, "Show Frequency");

                    ui.add(egui::Slider::new(&mut self.zoom, 0.25..=4.0).text("Zoom"));
                });
            });

            // Info panel
            ui.group(|ui| {
                ui.vertical(|ui| {
                    ui.heading("Symbol Info");
                    ui.add_space(8.0);

                    ui.label(format!("Symbol Value: {}", self.selected_symbol));
                    ui.label(format!(
                        "Binary: {:0width$b}",
                        self.selected_symbol,
                        width = params.sf.value() as usize
                    ));
                    ui.label(format!("Cyclic Shift: {} chips", self.selected_symbol));

                    ui.add_space(8.0);
                    ui.label(format!("Samples/Symbol: {}", params.samples_per_symbol()));
                    ui.label(format!("Duration: {:.3} ms", params.symbol_duration() * 1000.0));
                });
            });
        });

        ui.separator();

        // Generate chirp data
        let chirp_gen = ChirpGenerator::new(params.clone());
        let chirp = chirp_gen.generate_symbol_chirp_fast(self.selected_symbol);
        let frequencies = chirp_gen.compute_instantaneous_frequency(&chirp);

        let n = chirp.len();
        let duration_ms = params.symbol_duration() * 1000.0;

        // Time axis
        let time: Vec<f64> = (0..n).map(|i| i as f64 / n as f64 * duration_ms).collect();

        // Plot area
        ui.columns(if self.show_freq { 2 } else { 1 }, |columns| {
            // I/Q Plot
            columns[0].heading("Time Domain (I/Q Components)");

            let plot = Plot::new("iq_plot")
                .height(300.0)
                .allow_zoom(true)
                .allow_drag(true)
                .x_axis_label("Time (ms)")
                .y_axis_label("Amplitude");

            plot.show(&mut columns[0], |plot_ui| {
                if self.show_i {
                    let i_points: PlotPoints = time
                        .iter()
                        .enumerate()
                        .map(|(i, &t)| [t * self.zoom as f64, chirp[i].re])
                        .collect();
                    plot_ui.line(Line::new(i_points).name("I (Real)").color(egui::Color32::BLUE));
                }

                if self.show_q {
                    let q_points: PlotPoints = time
                        .iter()
                        .enumerate()
                        .map(|(i, &t)| [t * self.zoom as f64, chirp[i].im])
                        .collect();
                    plot_ui.line(Line::new(q_points).name("Q (Imag)").color(egui::Color32::RED));
                }
            });

            // Frequency Plot
            if self.show_freq && columns.len() > 1 {
                columns[1].heading("Instantaneous Frequency");

                let freq_plot = Plot::new("freq_plot")
                    .height(300.0)
                    .allow_zoom(true)
                    .allow_drag(true)
                    .x_axis_label("Time (ms)")
                    .y_axis_label("Frequency (Hz)");

                freq_plot.show(&mut columns[1], |plot_ui| {
                    let freq_points: PlotPoints = time[..frequencies.len()]
                        .iter()
                        .enumerate()
                        .map(|(i, &t)| [t * self.zoom as f64, frequencies[i]])
                        .collect();
                    plot_ui.line(
                        Line::new(freq_points)
                            .name("Frequency")
                            .color(egui::Color32::GREEN),
                    );
                });
            }
        });

        // Explanation
        ui.add_space(12.0);
        ui.separator();

        ui.collapsing("How Chirp Modulation Works", |ui| {
            ui.label(
                "In LoRa, each symbol is encoded as a chirp with a specific cyclic shift. \
                The base chirp (symbol 0) sweeps from -BW/2 to +BW/2. Higher symbol values \
                shift the starting frequency, causing the chirp to 'wrap around' earlier.",
            );
            ui.add_space(8.0);
            ui.label(
                "At the receiver, the signal is multiplied by a reference downchirp. \
                This converts the chirp into a single-frequency tone, whose frequency \
                (detected via FFT) indicates the symbol value.",
            );
        });

        ui.collapsing("Compare Symbols", |ui| {
            ui.label("Try different symbol values to see how the frequency sweep changes:");
            ui.label("• Symbol 0: Base chirp, no shift");
            ui.label(format!(
                "• Symbol {}: Maximum shift (half)",
                params.chips_per_symbol() / 2
            ));
            ui.label(format!(
                "• Symbol {}: Near maximum",
                params.chips_per_symbol() - 1
            ));
        });
    }
}
