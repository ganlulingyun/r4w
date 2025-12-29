//! Constellation diagram view

use egui::Ui;
use egui_plot::{Plot, PlotPoints, Points};
use r4w_core::types::IQSample;

pub struct ConstellationView {
    num_points: usize,
    start_offset: usize,
    point_size: f32,
    show_trajectory: bool,
}

impl ConstellationView {
    pub fn new() -> Self {
        Self {
            num_points: 1000,
            start_offset: 0,
            point_size: 2.0,
            show_trajectory: false,
        }
    }

    pub fn render(&mut self, ui: &mut Ui, waveform_name: &str, samples: &Option<Vec<IQSample>>) {
        ui.heading("I/Q Constellation Diagram");
        ui.add_space(8.0);

        ui.label(format!(
            "The constellation diagram plots the I (real) vs Q (imaginary) components \
            of the {} signal. The pattern reveals the modulation characteristics.",
            waveform_name
        ));

        ui.add_space(12.0);

        // Controls
        ui.horizontal(|ui| {
            ui.label("Points to display:");
            ui.add(egui::Slider::new(&mut self.num_points, 100..=10000).logarithmic(true));
        });

        ui.horizontal(|ui| {
            ui.label("Point size:");
            ui.add(egui::Slider::new(&mut self.point_size, 0.5..=10.0));

            ui.checkbox(&mut self.show_trajectory, "Show trajectory");
        });

        if let Some(ref signal) = samples {
            if signal.is_empty() {
                ui.label("Signal is empty. Generate a signal to see its constellation.");
                return;
            }

            // Adjust num_points slider range based on available samples
            let max_points = signal.len().max(1);
            self.num_points = self.num_points.min(max_points);

            let max_offset = signal.len().saturating_sub(self.num_points);
            self.start_offset = self.start_offset.min(max_offset);

            if max_offset > 0 {
                ui.add(
                    egui::Slider::new(&mut self.start_offset, 0..=max_offset)
                        .text("Start offset")
                        .clamping(egui::SliderClamping::Always),
                );
            }

            let end = (self.start_offset + self.num_points).min(signal.len());
            let display_samples = &signal[self.start_offset..end];

            // Show info about sample count
            if signal.len() < 10 {
                ui.colored_label(
                    egui::Color32::YELLOW,
                    format!("Note: Only {} samples available. Consider generating a longer signal.", signal.len()),
                );
                ui.add_space(8.0);
            }

            // Constellation plot
            let plot = Plot::new("constellation")
                .height(400.0)
                .width(400.0)
                .data_aspect(1.0)
                .allow_zoom(true)
                .allow_drag(true)
                .x_axis_label("I (In-phase)")
                .y_axis_label("Q (Quadrature)");

            plot.show(ui, |plot_ui| {
                let points: PlotPoints = display_samples
                    .iter()
                    .map(|s| [s.re, s.im])
                    .collect();

                plot_ui.points(
                    Points::new(points)
                        .name("I/Q")
                        .radius(self.point_size)
                        .color(egui::Color32::LIGHT_BLUE),
                );

                // Add unit circle for reference
                let circle_points: PlotPoints = (0..=100)
                    .map(|i| {
                        let theta = i as f64 * 2.0 * std::f64::consts::PI / 100.0;
                        [theta.cos(), theta.sin()]
                    })
                    .collect();

                plot_ui.line(
                    egui_plot::Line::new(circle_points)
                        .name("Unit Circle")
                        .color(egui::Color32::from_rgba_unmultiplied(128, 128, 128, 100))
                        .style(egui_plot::LineStyle::Dashed { length: 5.0 }),
                );
            });

            // Statistics (with guard against empty samples)
            ui.add_space(12.0);

            if !display_samples.is_empty() {
                let avg_magnitude: f64 =
                    display_samples.iter().map(|s| s.norm()).sum::<f64>() / display_samples.len() as f64;
                let max_magnitude = display_samples.iter().map(|s| s.norm()).fold(0.0_f64, f64::max);
                let min_magnitude = display_samples
                    .iter()
                    .map(|s| s.norm())
                    .fold(f64::MAX, f64::min);

                ui.horizontal(|ui| {
                    ui.label(format!("Avg magnitude: {:.4}", avg_magnitude));
                    ui.separator();
                    ui.label(format!("Min: {:.4}", min_magnitude));
                    ui.separator();
                    ui.label(format!("Max: {:.4}", max_magnitude));
                });

                // Calculate and display DC offset
                let dc_i: f64 = display_samples.iter().map(|s| s.re).sum::<f64>() / display_samples.len() as f64;
                let dc_q: f64 = display_samples.iter().map(|s| s.im).sum::<f64>() / display_samples.len() as f64;

                ui.label(format!("DC offset: I={:.4}, Q={:.4}", dc_i, dc_q));

                ui.label(format!("Displaying {} of {} samples", display_samples.len(), signal.len()));
            }
        } else {
            ui.label("Generate a signal to see its constellation.");
        }

        ui.add_space(20.0);
        ui.separator();

        ui.collapsing("Understanding the Constellation", |ui| {
            ui.label("• Constant-envelope signals (FM, FSK, chirp) form circles");
            ui.label("• PSK signals show distinct phase points on a circle");
            ui.label("• QAM signals show a grid of amplitude/phase combinations");
            ui.label("• The unit circle shows magnitude = 1 reference");
            ui.label("• Noise causes constellation points to scatter");
        });
    }
}
