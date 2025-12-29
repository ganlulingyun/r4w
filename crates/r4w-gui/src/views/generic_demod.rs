//! Generic Demodulation View
//!
//! Displays the demodulation pipeline for any waveform that implements the Waveform trait.

use egui::{Color32, Ui};
use egui_plot::{Line, Plot, PlotPoints};
use r4w_core::types::IQSample;
use r4w_core::waveform::{DemodulationStep, Waveform};

/// Generic demodulation view that works with any waveform
pub struct GenericDemodulationView {
    /// Currently selected step index
    selected_step: usize,
    /// Show I component in plots
    show_i: bool,
    /// Show Q component in plots
    show_q: bool,
    /// Cached steps from last demodulation
    cached_steps: Vec<DemodulationStep>,
    /// Last waveform name (to detect changes)
    last_waveform: String,
}

impl Default for GenericDemodulationView {
    fn default() -> Self {
        Self::new()
    }
}

impl GenericDemodulationView {
    pub fn new() -> Self {
        Self {
            selected_step: 0,
            show_i: true,
            show_q: true,
            cached_steps: Vec::new(),
            last_waveform: String::new(),
        }
    }

    /// Render the generic demodulation view
    pub fn render(
        &mut self,
        ui: &mut Ui,
        waveform: &dyn Waveform,
        samples: &Option<Vec<IQSample>>,
    ) {
        let info = waveform.info();

        ui.heading(format!("{} Demodulation Pipeline", info.name));
        ui.add_space(4.0);

        ui.label(format!(
            "{} - {}",
            info.full_name, info.description
        ));

        ui.add_space(8.0);

        // Update steps if we have samples and waveform changed
        if let Some(ref signal) = samples {
            if self.last_waveform != info.name || self.cached_steps.is_empty() {
                self.cached_steps = waveform.get_demodulation_steps(signal);
                self.last_waveform = info.name.to_string();
                self.selected_step = 0;
            }
        }

        // Pipeline overview diagram
        self.render_pipeline_diagram(ui, &info.name);

        ui.add_space(12.0);
        ui.separator();

        // Step selector
        if !self.cached_steps.is_empty() {
            ui.horizontal(|ui| {
                ui.label("Demodulation Step:");
                egui::ComboBox::from_id_salt("generic_demod_step")
                    .selected_text(if self.selected_step < self.cached_steps.len() {
                        &self.cached_steps[self.selected_step].name
                    } else {
                        "Select..."
                    })
                    .show_ui(ui, |ui| {
                        for (i, step) in self.cached_steps.iter().enumerate() {
                            ui.selectable_value(&mut self.selected_step, i, &step.name);
                        }
                    });
            });

            ui.add_space(8.0);

            // Display selected step
            if self.selected_step < self.cached_steps.len() {
                self.render_step(ui, &self.cached_steps[self.selected_step].clone());
            }
        } else if samples.is_none() {
            ui.label("Generate a signal to see demodulation steps.");
            ui.small("Use the 'Generate' button in the sidebar parameters.");
        }

        ui.add_space(12.0);
        ui.separator();

        // Input signal display
        self.render_input_signal(ui, samples, waveform);
    }

    /// Render a visual pipeline diagram
    fn render_pipeline_diagram(&self, ui: &mut Ui, waveform_name: &str) {
        ui.collapsing("Demodulation Process", |ui| {
            let diagram = match waveform_name {
                "BPSK" | "QPSK" | "8-PSK" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│  Carrier      │───►│   Symbol    │───►│  Output  │
│ Samples  │    │  Recovery     │    │  Detection  │    │   Bits   │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
               Phase Tracking         Constellation
               (PLL or DPLL)          Decision Regions"#,

                "16-QAM" | "64-QAM" | "256-QAM" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│  AGC +        │───►│  Symbol     │───►│  Output  │
│ Samples  │    │  Carrier Sync │    │  Slicer     │    │   Bits   │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
              Amplitude + Phase        I/Q Decision
              Recovery                 Thresholds"#,

                "BFSK" | "4-FSK" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│  Frequency    │───►│  Symbol     │───►│  Output  │
│ Samples  │    │  Discriminator│    │  Detection  │    │   Bits   │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
               FM Demod or            Frequency Bin
               Matched Filters        Decision"#,

                "OOK" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│   Envelope    │───►│  Threshold  │───►│  Output  │
│ Samples  │    │   Detection   │    │  Comparison │    │   Bits   │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
               |I + jQ|              Adaptive or
               Magnitude             Fixed Threshold"#,

                "AM" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│   Envelope    │───►│   Low-pass  │───►│  Output  │
│ Samples  │    │   Detector    │    │   Filter    │    │  Signal  │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
               |I + jQ|              Remove carrier
               Rectification         frequency"#,

                "FM" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│  Frequency    │───►│   Low-pass  │───►│  Output  │
│ Samples  │    │  Discriminator│    │   Filter    │    │  Signal  │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
               d(phase)/dt           De-emphasis
               Phase derivative      (optional)"#,

                "OFDM" => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│  Remove CP    │───►│    FFT      │───►│  Output  │
│ Samples  │    │  + Sync       │    │ per subcarr │    │   Bits   │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘
                     │                      │
                     ▼                      ▼
               Symbol Timing         Per-subcarrier
               Recovery              QAM/PSK demod"#,

                _ => r#"
┌──────────┐    ┌───────────────┐    ┌─────────────┐    ┌──────────┐
│  Input   │───►│   Symbol      │───►│    Bit      │───►│  Output  │
│ Samples  │    │   Detection   │    │   Recovery  │    │   Bits   │
└──────────┘    └───────────────┘    └─────────────┘    └──────────┘"#,
            };
            ui.code(diagram);
        });
    }

    /// Render a single demodulation step
    fn render_step(&self, ui: &mut Ui, step: &DemodulationStep) {
        ui.group(|ui| {
            ui.heading(&step.name);
            ui.label(&step.description);

            // Show decision info if available
            if let Some(ref info) = step.decision_info {
                ui.add_space(4.0);
                ui.small(format!("Decision: {}", info));
            }

            // Show confidence if available
            if let Some(confidence) = step.confidence {
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    ui.label("Confidence/SNR:");
                    let color = if confidence > 10.0 {
                        Color32::GREEN
                    } else if confidence > 5.0 {
                        Color32::YELLOW
                    } else {
                        Color32::RED
                    };
                    ui.colored_label(color, format!("{:.1} dB", confidence));
                });
            }

            // Show input samples
            if let Some(ref samples) = step.input_samples {
                if !samples.is_empty() {
                    ui.add_space(8.0);
                    ui.label(format!("Input Samples ({}):", samples.len()));

                    let plot = Plot::new(format!("step_samples_{}", step.name))
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

            // Show detected symbols
            if let Some(ref symbols) = step.detected_symbols {
                ui.add_space(8.0);
                ui.label(format!("Detected Symbols ({}):", symbols.len()));

                let sym_str: String = symbols
                    .iter()
                    .take(30)
                    .map(|s| format!("{}", s))
                    .collect::<Vec<_>>()
                    .join(", ");
                ui.code(&sym_str);

                if symbols.len() > 30 {
                    ui.small("...");
                }
            }

            // Show recovered bits
            if let Some(ref bits) = step.recovered_bits {
                ui.add_space(8.0);
                ui.label(format!("Recovered Bits ({} bytes):", bits.len()));

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

                // Try to show as ASCII
                let ascii: String = bits
                    .iter()
                    .take(32)
                    .map(|&b| if b.is_ascii_graphic() || b == b' ' { b as char } else { '.' })
                    .collect();
                ui.small(format!("ASCII: {}", ascii));

                if bits.len() > 16 {
                    ui.small("...");
                }
            }
        });
    }

    /// Render the input signal display
    fn render_input_signal(
        &mut self,
        ui: &mut Ui,
        samples: &Option<Vec<IQSample>>,
        waveform: &dyn Waveform,
    ) {
        ui.heading("Input Signal for Demodulation");

        ui.horizontal(|ui| {
            ui.checkbox(&mut self.show_i, "I (In-phase)");
            ui.checkbox(&mut self.show_q, "Q (Quadrature)");
        });

        if let Some(ref signal) = samples {
            let plot = Plot::new("demod_input_signal")
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
            ui.label("Generate a signal to see the demodulation input.");
            ui.small("Use the 'Generate' button in the sidebar parameters.");
        }
    }
}
