//! Modulation process visualization

use egui::Ui;
use egui_plot::{Line, Plot, PlotPoints};
use r4w_core::modulation::Modulator;
use r4w_core::params::LoRaParams;
use r4w_core::types::IQSample;

pub struct ModulationView {
    show_preamble: bool,
    show_payload: bool,
    selected_stage: usize,
}

impl ModulationView {
    pub fn new() -> Self {
        Self {
            show_preamble: true,
            show_payload: true,
            selected_stage: 0,
        }
    }

    pub fn render(
        &mut self,
        ui: &mut Ui,
        params: &LoRaParams,
        samples: &Option<Vec<IQSample>>,
        modulator: Option<&Modulator>,
    ) {
        ui.heading("Modulation Pipeline");
        ui.add_space(8.0);

        ui.label(
            "The modulation process converts raw data bytes into I/Q samples \
            ready for transmission.",
        );

        ui.add_space(12.0);

        // Stage selector
        if let Some(mod_ref) = modulator {
            let stages = mod_ref.stages();
            if !stages.is_empty() {
                ui.horizontal(|ui| {
                    ui.label("Pipeline Stage:");
                    egui::ComboBox::from_id_salt("stage_select")
                        .selected_text(if self.selected_stage < stages.len() {
                            &stages[self.selected_stage].name
                        } else {
                            "Select..."
                        })
                        .show_ui(ui, |ui| {
                            for (i, stage) in stages.iter().enumerate() {
                                ui.selectable_value(&mut self.selected_stage, i, &stage.name);
                            }
                        });
                });

                if self.selected_stage < stages.len() {
                    let stage = &stages[self.selected_stage];

                    ui.add_space(8.0);
                    ui.group(|ui| {
                        ui.heading(&stage.name);
                        ui.label(&stage.description);

                        // Show stage data
                        if let Some(ref bits) = stage.bits {
                            ui.add_space(8.0);
                            ui.label(format!("Data ({} bytes):", bits.len()));

                            // Show as hex
                            let hex: String = bits
                                .iter()
                                .take(32)
                                .map(|b| format!("{:02X}", b))
                                .collect::<Vec<_>>()
                                .join(" ");
                            ui.code(&hex);
                            if bits.len() > 32 {
                                ui.label("...");
                            }
                        }

                        if let Some(ref symbols) = stage.symbols {
                            ui.add_space(8.0);
                            ui.label(format!("Symbols ({}):", symbols.len()));

                            let sym_str: String = symbols
                                .iter()
                                .take(16)
                                .map(|s| format!("{}", s))
                                .collect::<Vec<_>>()
                                .join(", ");
                            ui.code(&sym_str);
                            if symbols.len() > 16 {
                                ui.label("...");
                            }
                        }

                        // Plot time domain if available
                        if let Some(ref td) = stage.time_domain {
                            ui.add_space(8.0);
                            ui.label(format!("Samples: {}", td.len()));

                            let plot = Plot::new(format!("stage_plot_{}", self.selected_stage))
                                .height(200.0)
                                .allow_zoom(true)
                                .x_axis_label("Sample")
                                .y_axis_label("Amplitude");

                            let display_samples = td.len().min(2000);

                            plot.show(ui, |plot_ui| {
                                let i_points: PlotPoints = (0..display_samples)
                                    .map(|i| [i as f64, td[i].re])
                                    .collect();
                                plot_ui.line(
                                    Line::new(i_points)
                                        .name("I")
                                        .color(egui::Color32::BLUE),
                                );

                                let q_points: PlotPoints = (0..display_samples)
                                    .map(|i| [i as f64, td[i].im])
                                    .collect();
                                plot_ui.line(
                                    Line::new(q_points)
                                        .name("Q")
                                        .color(egui::Color32::RED),
                                );
                            });
                        }
                    });
                }
            }
        }

        ui.add_space(12.0);
        ui.separator();

        // Full signal plot
        if let Some(ref signal) = samples {
            ui.heading("Complete Modulated Signal");

            ui.horizontal(|ui| {
                ui.checkbox(&mut self.show_preamble, "Show Preamble");
                ui.checkbox(&mut self.show_payload, "Show Payload");
            });

            let preamble_samples = params.samples_per_symbol() * (params.preamble_length + 4);

            let plot = Plot::new("full_signal_plot")
                .height(250.0)
                .allow_zoom(true)
                .allow_drag(true)
                .x_axis_label("Sample Index")
                .y_axis_label("Amplitude");

            let display_len = signal.len().min(10000);

            plot.show(ui, |plot_ui| {
                let i_points: PlotPoints = (0..display_len)
                    .filter(|&i| {
                        (self.show_preamble && i < preamble_samples)
                            || (self.show_payload && i >= preamble_samples)
                    })
                    .map(|i| [i as f64, signal[i].re])
                    .collect();

                plot_ui.line(Line::new(i_points).name("I").color(egui::Color32::BLUE));

                let q_points: PlotPoints = (0..display_len)
                    .filter(|&i| {
                        (self.show_preamble && i < preamble_samples)
                            || (self.show_payload && i >= preamble_samples)
                    })
                    .map(|i| [i as f64, signal[i].im])
                    .collect();

                plot_ui.line(Line::new(q_points).name("Q").color(egui::Color32::RED));
            });

            ui.label(format!(
                "Total samples: {} | Preamble: {} | Payload: {}",
                signal.len(),
                preamble_samples,
                signal.len().saturating_sub(preamble_samples)
            ));
        } else {
            ui.label("Generate a signal to see the modulation output.");
        }
    }
}
