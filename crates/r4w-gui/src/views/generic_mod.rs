//! Generic Modulation View
//!
//! Displays the modulation pipeline for any waveform that implements the Waveform trait.

use egui::{Color32, Ui};
use egui_plot::{Line, Plot, PlotPoints, Points};
use r4w_core::types::IQSample;
use r4w_core::waveform::{ModulationStage, Waveform};

/// Generic modulation view that works with any waveform
pub struct GenericModulationView {
    /// Currently selected stage index
    selected_stage: usize,
    /// Show I component in plots
    show_i: bool,
    /// Show Q component in plots
    show_q: bool,
    /// Cached stages from last modulation
    cached_stages: Vec<ModulationStage>,
    /// Last waveform name (to detect changes)
    last_waveform: String,
}

impl Default for GenericModulationView {
    fn default() -> Self {
        Self::new()
    }
}

impl GenericModulationView {
    pub fn new() -> Self {
        Self {
            selected_stage: 0,
            show_i: true,
            show_q: true,
            cached_stages: Vec::new(),
            last_waveform: String::new(),
        }
    }

    /// Render the generic modulation view
    pub fn render(
        &mut self,
        ui: &mut Ui,
        waveform: &dyn Waveform,
        test_data: &[u8],
        samples: &Option<Vec<IQSample>>,
    ) {
        let info = waveform.info();

        ui.heading(format!("{} Modulation Pipeline", info.name));
        ui.add_space(4.0);

        ui.label(format!(
            "{} - {}",
            info.full_name, info.description
        ));

        ui.add_space(8.0);

        // Update stages if waveform changed or no stages cached
        if self.last_waveform != info.name || self.cached_stages.is_empty() {
            self.cached_stages = waveform.get_modulation_stages(test_data);
            self.last_waveform = info.name.to_string();
            self.selected_stage = 0;
        }

        // Pipeline overview diagram
        self.render_pipeline_diagram(ui, &info.name);

        ui.add_space(12.0);
        ui.separator();

        // Stage selector
        if !self.cached_stages.is_empty() {
            ui.horizontal(|ui| {
                ui.label("Pipeline Stage:");
                egui::ComboBox::from_id_salt("generic_mod_stage")
                    .selected_text(if self.selected_stage < self.cached_stages.len() {
                        &self.cached_stages[self.selected_stage].name
                    } else {
                        "Select..."
                    })
                    .show_ui(ui, |ui| {
                        for (i, stage) in self.cached_stages.iter().enumerate() {
                            ui.selectable_value(&mut self.selected_stage, i, &stage.name);
                        }
                    });
            });

            ui.add_space(8.0);

            // Display selected stage
            if self.selected_stage < self.cached_stages.len() {
                self.render_stage(ui, &self.cached_stages[self.selected_stage].clone());
            }
        }

        ui.add_space(12.0);
        ui.separator();

        // Full signal display
        self.render_full_signal(ui, samples, waveform);
    }

    /// Render a visual pipeline diagram
    fn render_pipeline_diagram(&self, ui: &mut Ui, waveform_name: &str) {
        ui.collapsing("Modulation Process", |ui| {
            let diagram = match waveform_name {
                "BPSK" | "QPSK" | "8-PSK" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│ Bit-to-Symbol │───►│Constellation│───►│  Output  │
│  Bits    │    │   Mapping     │    │  Mapping    │    │ Samples  │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
                Gray Coding           Phase Assignment
                (optional)            (0°, 90°, 180°, 270°)"#,

                "16-QAM" | "64-QAM" | "256-QAM" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│ Bit-to-Symbol │───►│     QAM     │───►│  Output  │
│  Bits    │    │   Mapping     │    │Constellation│    │ Samples  │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
                Gray Coding           I + jQ mapping
              (minimize errors)     (amplitude + phase)"#,

                "BFSK" | "4-FSK" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│ Bit-to-Symbol │───►│  Frequency  │───►│  Output  │
│  Bits    │    │   Mapping     │    │  Generator  │    │ Samples  │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
               Symbol Value          f = f_c ± Δf
               (0,1 or 0-3)        (continuous phase)"#,

                "OOK" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│  Bit Mapping  │───►│   Carrier   │───►│  Output  │
│  Bits    │    │   (1 or 0)    │    │  On/Off     │    │ Samples  │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
                 0 = Off               s(t) = A·cos(ωt)
                 1 = On                  or 0"#,

                "AM" | "FM" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│   Message     │───►│  Carrier    │───►│  Output  │
│  Signal  │    │   Shaping     │    │ Modulation  │    │ Samples  │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
              Baseband Signal       AM: A(t)·cos(ωt)
                                   FM: cos(ω + Δω·m(t))"#,

                _ => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│ Bit-to-Symbol │───►│   Sample    │───►│  Output  │
│  Bits    │    │   Mapping     │    │ Generation  │    │ Samples  │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘"#,
            };
            ui.code(diagram);
        });
    }

    /// Render a single modulation stage
    fn render_stage(&self, ui: &mut Ui, stage: &ModulationStage) {
        ui.group(|ui| {
            ui.heading(&stage.name);
            ui.label(&stage.description);

            // Show input bits
            if let Some(ref bits) = stage.input_bits {
                ui.add_space(8.0);
                ui.label(format!("Input Bits ({} bytes):", bits.len()));

                // Show as binary
                let binary: String = bits
                    .iter()
                    .take(8)
                    .map(|b| format!("{:08b}", b))
                    .collect::<Vec<_>>()
                    .join(" ");
                ui.code(&binary);

                // Show as hex
                let hex: String = bits
                    .iter()
                    .take(16)
                    .map(|b| format!("{:02X}", b))
                    .collect::<Vec<_>>()
                    .join(" ");
                ui.small(format!("Hex: {}", hex));

                if bits.len() > 16 {
                    ui.small("...");
                }
            }

            // Show output symbols
            if let Some(ref symbols) = stage.output_symbols {
                ui.add_space(8.0);
                ui.label(format!("Output Symbols ({}):", symbols.len()));

                let sym_str: String = symbols
                    .iter()
                    .take(20)
                    .map(|s| format!("{}", s))
                    .collect::<Vec<_>>()
                    .join(", ");
                ui.code(&sym_str);

                if symbols.len() > 20 {
                    ui.small("...");
                }
            }

            // Show constellation if available
            if let Some(ref constellation) = stage.constellation {
                if !constellation.is_empty() {
                    ui.add_space(8.0);
                    ui.label("Constellation Diagram:");

                    let plot = Plot::new(format!("stage_constellation_{}", stage.name))
                        .height(200.0)
                        .width(200.0)
                        .data_aspect(1.0)
                        .x_axis_label("I (In-phase)")
                        .y_axis_label("Q (Quadrature)")
                        .allow_zoom(true)
                        .allow_drag(true);

                    plot.show(ui, |plot_ui| {
                        let points: PlotPoints = constellation
                            .iter()
                            .map(|c| [c.re, c.im])
                            .collect();
                        plot_ui.points(
                            Points::new(points)
                                .radius(6.0)
                                .color(Color32::from_rgb(100, 200, 100)),
                        );
                    });
                }
            }

            // Show time domain samples
            if let Some(ref samples) = stage.samples {
                if !samples.is_empty() {
                    ui.add_space(8.0);
                    ui.label(format!("I/Q Samples ({}):", samples.len()));

                    let plot = Plot::new(format!("stage_samples_{}", stage.name))
                        .height(150.0)
                        .x_axis_label("Sample")
                        .y_axis_label("Amplitude")
                        .allow_zoom(true)
                        .allow_drag(true);

                    let display_len = samples.len().min(1000);

                    plot.show(ui, |plot_ui| {
                        let i_points: PlotPoints = (0..display_len)
                            .map(|i| [i as f64, samples[i].re])
                            .collect();
                        plot_ui.line(
                            Line::new(i_points)
                                .name("I")
                                .color(Color32::BLUE),
                        );

                        let q_points: PlotPoints = (0..display_len)
                            .map(|i| [i as f64, samples[i].im])
                            .collect();
                        plot_ui.line(
                            Line::new(q_points)
                                .name("Q")
                                .color(Color32::RED),
                        );
                    });
                }
            }
        });
    }

    /// Render the full modulated signal
    fn render_full_signal(
        &mut self,
        ui: &mut Ui,
        samples: &Option<Vec<IQSample>>,
        waveform: &dyn Waveform,
    ) {
        ui.heading("Complete Modulated Signal");

        ui.horizontal(|ui| {
            ui.checkbox(&mut self.show_i, "I (In-phase)");
            ui.checkbox(&mut self.show_q, "Q (Quadrature)");
        });

        if let Some(ref signal) = samples {
            let plot = Plot::new("full_mod_signal")
                .height(200.0)
                .x_axis_label("Sample Index")
                .y_axis_label("Amplitude")
                .allow_zoom(true)
                .allow_drag(true)
                .legend(egui_plot::Legend::default());

            let display_len = signal.len().min(5000);

            plot.show(ui, |plot_ui| {
                if self.show_i {
                    let i_points: PlotPoints = (0..display_len)
                        .map(|i| [i as f64, signal[i].re])
                        .collect();
                    plot_ui.line(
                        Line::new(i_points)
                            .name("I")
                            .color(Color32::BLUE),
                    );
                }

                if self.show_q {
                    let q_points: PlotPoints = (0..display_len)
                        .map(|i| [i as f64, signal[i].im])
                        .collect();
                    plot_ui.line(
                        Line::new(q_points)
                            .name("Q")
                            .color(Color32::RED),
                    );
                }
            });

            let sps = waveform.samples_per_symbol();
            let num_symbols = if sps > 0 { signal.len() / sps } else { 0 };

            ui.label(format!(
                "Total: {} samples | {} samples/symbol | ~{} symbols",
                signal.len(),
                sps,
                num_symbols
            ));
        } else {
            ui.label("Generate a signal to see the modulation output.");
            ui.small("Use the 'Generate' button in the sidebar parameters.");
        }
    }
}
