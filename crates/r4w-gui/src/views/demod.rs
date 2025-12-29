//! Demodulation visualization view

use egui::Ui;
use egui_plot::{Line, Plot, PlotPoints};
use r4w_core::demodulation::Demodulator;
use r4w_core::params::LoRaParams;
use r4w_core::types::IQSample;
use r4w_sim::Channel;

pub struct DemodView {
    selected_symbol_idx: usize,
    show_mixed: bool,
    show_spectrum: bool,
}

impl DemodView {
    pub fn new() -> Self {
        Self {
            selected_symbol_idx: 0,
            show_mixed: true,
            show_spectrum: true,
        }
    }

    pub fn render(
        &mut self,
        ui: &mut Ui,
        params: &LoRaParams,
        samples: &Option<Vec<IQSample>>,
        channel: &mut Channel,
        mut demodulator: Option<&mut Demodulator>,
    ) {
        ui.heading("Demodulation Process");
        ui.add_space(8.0);

        ui.label(
            "Demodulation extracts symbols from received I/Q samples using \
            FFT-based detection after mixing with a reference downchirp.",
        );

        ui.add_space(12.0);

        // Process explanation
        ui.collapsing("How CSS Demodulation Works", |ui| {
            ui.code(
                r#"
1. Received chirp × conjugate(reference downchirp) = tone at symbol frequency
2. FFT finds the peak frequency
3. Peak bin index = transmitted symbol value

   Received:     Reference:      Result:
      /             \               |
     /               \              |  ← Single tone
    /                 \             |    at symbol freq
   /                   \            |
"#,
            );
        });

        ui.add_space(12.0);

        if let Some(ref signal) = samples {
            // Apply channel effects
            let noisy_signal = channel.apply(signal);

            let n = params.samples_per_symbol();
            let num_symbols = noisy_signal.len() / n;

            // Skip preamble for symbol display
            let preamble_symbols = params.preamble_length + 4; // upchirps + sync + downchirps
            let payload_start = preamble_symbols;

            ui.horizontal(|ui| {
                ui.label("Symbol Index:");
                let max_idx = num_symbols.saturating_sub(payload_start + 1);
                ui.add(egui::Slider::new(&mut self.selected_symbol_idx, 0..=max_idx));
            });

            ui.horizontal(|ui| {
                ui.checkbox(&mut self.show_mixed, "Show Mixed Signal");
                ui.checkbox(&mut self.show_spectrum, "Show FFT Spectrum");
            });

            // Get the selected symbol's samples
            let sym_start = (payload_start + self.selected_symbol_idx) * n;
            if sym_start + n <= noisy_signal.len() {
                let symbol_samples = &noisy_signal[sym_start..sym_start + n];

                // Demodulate this symbol
                if let Some(ref mut demod) = demodulator {
                    let result = demod.demodulate_symbol(symbol_samples);

                    ui.add_space(8.0);
                    ui.group(|ui| {
                        ui.heading(format!("Symbol {} Result", self.selected_symbol_idx));
                        ui.label(format!("Detected Symbol: {}", result.symbol));
                        ui.label(format!("Magnitude: {:.2}", result.magnitude));
                        ui.label(format!("Phase: {:.2}°", result.phase.to_degrees()));
                        if let Some(snr) = result.snr_estimate {
                            ui.label(format!("Est. SNR: {:.1} dB", snr));
                        }
                    });

                    // Show spectrum
                    if self.show_spectrum {
                        let spectrum = demod.get_symbol_spectrum(symbol_samples);

                        ui.add_space(12.0);
                        ui.heading("FFT Spectrum (Symbol Detection)");

                        let plot = Plot::new("demod_spectrum")
                            .height(250.0)
                            .allow_zoom(true)
                            .x_axis_label("Bin (Symbol Value)")
                            .y_axis_label("Magnitude");

                        plot.show(ui, |plot_ui| {
                            let points: PlotPoints = spectrum
                                .iter()
                                .enumerate()
                                .map(|(i, &mag)| [i as f64, mag])
                                .collect();

                            plot_ui.line(
                                Line::new(points)
                                    .name("Spectrum")
                                    .color(egui::Color32::GREEN),
                            );
                        });

                        ui.label(format!(
                            "Peak at bin {} = symbol {}",
                            result.symbol, result.symbol
                        ));
                    }
                }

                // Show time domain
                if self.show_mixed {
                    ui.add_space(12.0);
                    ui.heading("Received Symbol (Time Domain)");

                    let plot = Plot::new("symbol_time")
                        .height(200.0)
                        .allow_zoom(true)
                        .x_axis_label("Sample")
                        .y_axis_label("Amplitude");

                    plot.show(ui, |plot_ui| {
                        let i_points: PlotPoints = symbol_samples
                            .iter()
                            .enumerate()
                            .map(|(i, s)| [i as f64, s.re])
                            .collect();
                        plot_ui.line(Line::new(i_points).name("I").color(egui::Color32::BLUE));

                        let q_points: PlotPoints = symbol_samples
                            .iter()
                            .enumerate()
                            .map(|(i, s)| [i as f64, s.im])
                            .collect();
                        plot_ui.line(Line::new(q_points).name("Q").color(egui::Color32::RED));
                    });
                }
            } else {
                ui.label("Symbol index out of range");
            }
        } else {
            ui.label("Generate a signal first to see demodulation.");
        }
    }
}
