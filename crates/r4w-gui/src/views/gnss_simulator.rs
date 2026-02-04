//! GNSS Scenario Simulator View
//!
//! Interactive GUI for configuring and visualizing GNSS scenario generation
//! with sky plot, C/N0 bar chart, and IQ waveform display.

use egui::{Color32, Pos2, Stroke, Ui, Vec2};
use r4w_core::waveform::gnss::scenario::GnssScenario;
use r4w_core::waveform::gnss::scenario_config::{GnssScenarioPreset, GnssScenarioConfig};
use r4w_core::waveform::gnss::satellite_emitter::SatelliteStatus;
use r4w_core::types::IQSample;
use std::f64::consts::PI;

// trace:FR-042 | ai:claude
pub struct GnssSimulatorView {
    /// Selected preset
    selected_preset: usize,
    /// Generated satellite statuses
    statuses: Vec<SatelliteStatus>,
    /// Generated IQ samples (last block)
    iq_samples: Vec<IQSample>,
    /// Whether scenario has been generated
    generated: bool,
    /// Duration in ms
    duration_ms: f32,
    /// Sample rate in MHz
    sample_rate_mhz: f32,
    /// Enable ionosphere
    iono_enabled: bool,
    /// Enable troposphere
    tropo_enabled: bool,
    /// Enable multipath
    multipath_enabled: bool,
    /// Average power of generated signal
    avg_power_db: f64,
    /// Number of samples generated
    num_samples: usize,
}

impl GnssSimulatorView {
    pub fn new() -> Self {
        Self {
            selected_preset: 0,
            statuses: Vec::new(),
            iq_samples: Vec::new(),
            generated: false,
            duration_ms: 1.0,
            sample_rate_mhz: 2.046,
            iono_enabled: true,
            tropo_enabled: true,
            multipath_enabled: false,
            avg_power_db: 0.0,
            num_samples: 0,
        }
    }

    pub fn render(&mut self, ui: &mut Ui) {
        ui.heading("GNSS Scenario Simulator");
        ui.label("Generate multi-satellite GNSS IQ signals with realistic channel effects");
        ui.add_space(8.0);

        // Controls
        ui.horizontal(|ui| {
            // Left: Configuration
            ui.vertical(|ui| {
                ui.set_min_width(250.0);
                ui.group(|ui| {
                    ui.heading("Configuration");
                    ui.add_space(4.0);

                    // Preset selector
                    let presets = GnssScenarioPreset::all();
                    let preset_names: Vec<String> = presets.iter().map(|p| format!("{}", p)).collect();
                    egui::ComboBox::from_label("Preset")
                        .selected_text(&preset_names[self.selected_preset])
                        .show_ui(ui, |ui| {
                            for (i, name) in preset_names.iter().enumerate() {
                                ui.selectable_value(&mut self.selected_preset, i, name);
                            }
                        });

                    ui.add_space(4.0);
                    ui.add(egui::Slider::new(&mut self.duration_ms, 0.1..=100.0)
                        .text("Duration (ms)")
                        .logarithmic(true));
                    ui.add(egui::Slider::new(&mut self.sample_rate_mhz, 1.0..=20.0)
                        .text("Sample Rate (MHz)"));

                    ui.add_space(4.0);
                    ui.heading("Environment");
                    ui.checkbox(&mut self.iono_enabled, "Ionosphere (Klobuchar)");
                    ui.checkbox(&mut self.tropo_enabled, "Troposphere (Saastamoinen)");
                    ui.checkbox(&mut self.multipath_enabled, "Multipath");

                    ui.add_space(8.0);
                    if ui.button("Generate Scenario").clicked() {
                        self.run_scenario();
                    }
                });

                // Stats
                if self.generated {
                    ui.add_space(4.0);
                    ui.group(|ui| {
                        ui.heading("Output");
                        ui.label(format!("Samples: {}", self.num_samples));
                        ui.label(format!("Avg Power: {:.1} dB", self.avg_power_db));
                        ui.label(format!("Satellites: {}", self.statuses.len()));
                        let visible = self.statuses.iter().filter(|s| s.visible).count();
                        ui.label(format!("Visible: {}", visible));
                    });
                }
            });

            ui.add_space(8.0);

            // Right: Visualizations
            ui.vertical(|ui| {
                if self.generated && !self.statuses.is_empty() {
                    // Sky plot and C/N0 side by side
                    ui.horizontal(|ui| {
                        self.render_sky_plot(ui);
                        ui.add_space(8.0);
                        self.render_cn0_bars(ui);
                    });
                    ui.add_space(8.0);
                    self.render_iq_waveform(ui);
                } else {
                    ui.centered_and_justified(|ui| {
                        ui.label("Configure and click 'Generate Scenario' to visualize");
                    });
                }
            });
        });
    }

    fn run_scenario(&mut self) {
        let presets = GnssScenarioPreset::all();
        let preset = presets[self.selected_preset];
        let mut config = preset.to_config();

        config.output.duration_s = self.duration_ms as f64 / 1000.0;
        config.output.sample_rate = self.sample_rate_mhz as f64 * 1e6;
        config.environment.ionosphere_enabled = self.iono_enabled;
        config.environment.troposphere_enabled = self.tropo_enabled;
        config.environment.multipath_enabled = self.multipath_enabled;

        let mut scenario = GnssScenario::new(config);
        self.statuses = scenario.satellite_status();
        let samples = scenario.generate();

        // Keep last 4096 samples for display
        let display_samples = if samples.len() > 4096 {
            samples[samples.len()-4096..].to_vec()
        } else {
            samples.clone()
        };

        self.num_samples = samples.len();
        if !samples.is_empty() {
            let avg: f64 = samples.iter()
                .map(|s| s.re * s.re + s.im * s.im)
                .sum::<f64>() / samples.len() as f64;
            self.avg_power_db = if avg > 0.0 { 10.0 * avg.log10() } else { -999.0 };
        }

        self.iq_samples = display_samples;
        self.generated = true;
    }

    fn render_sky_plot(&self, ui: &mut Ui) {
        let size = 200.0;
        let (response, painter) = ui.allocate_painter(Vec2::new(size, size), egui::Sense::hover());
        let center = response.rect.center();
        let radius = size / 2.0 - 10.0;

        // Background circles (elevation rings)
        for el in [0, 30, 60, 90] {
            let r = radius * (90.0 - el as f32) / 90.0;
            painter.circle_stroke(center, r, Stroke::new(0.5, Color32::GRAY));
        }

        // Crosshairs
        painter.line_segment(
            [Pos2::new(center.x - radius, center.y), Pos2::new(center.x + radius, center.y)],
            Stroke::new(0.5, Color32::DARK_GRAY),
        );
        painter.line_segment(
            [Pos2::new(center.x, center.y - radius), Pos2::new(center.x, center.y + radius)],
            Stroke::new(0.5, Color32::DARK_GRAY),
        );

        // Cardinal labels
        let font = egui::FontId::proportional(10.0);
        painter.text(Pos2::new(center.x, center.y - radius - 5.0), egui::Align2::CENTER_BOTTOM, "N", font.clone(), Color32::WHITE);
        painter.text(Pos2::new(center.x + radius + 5.0, center.y), egui::Align2::LEFT_CENTER, "E", font.clone(), Color32::WHITE);
        painter.text(Pos2::new(center.x, center.y + radius + 5.0), egui::Align2::CENTER_TOP, "S", font.clone(), Color32::WHITE);
        painter.text(Pos2::new(center.x - radius - 5.0, center.y), egui::Align2::RIGHT_CENTER, "W", font.clone(), Color32::WHITE);

        // Plot satellites
        for status in &self.statuses {
            let el = status.elevation_deg as f32;
            let az = status.azimuth_deg as f32;

            let r = radius * (90.0 - el) / 90.0;
            let angle = (az - 90.0).to_radians(); // 0=North, rotate to screen coords
            let x = center.x + r * angle.cos();
            let y = center.y + r * angle.sin();

            let color = if status.visible {
                if status.cn0_dbhz > 35.0 { Color32::GREEN }
                else if status.cn0_dbhz > 25.0 { Color32::YELLOW }
                else { Color32::RED }
            } else {
                Color32::DARK_GRAY
            };

            painter.circle_filled(Pos2::new(x, y), 6.0, color);
            let label_font = egui::FontId::proportional(8.0);
            painter.text(
                Pos2::new(x, y - 8.0),
                egui::Align2::CENTER_BOTTOM,
                format!("{}", status.prn),
                label_font,
                Color32::WHITE,
            );
        }

        // Title
        let title_font = egui::FontId::proportional(11.0);
        painter.text(
            Pos2::new(center.x, response.rect.min.y + 2.0),
            egui::Align2::CENTER_TOP,
            "Sky Plot",
            title_font,
            Color32::WHITE,
        );
    }

    fn render_cn0_bars(&self, ui: &mut Ui) {
        let visible: Vec<&SatelliteStatus> = self.statuses.iter().filter(|s| s.visible).collect();
        if visible.is_empty() { return; }

        let width = (visible.len() as f32 * 30.0).max(100.0);
        let height = 200.0;
        let (response, painter) = ui.allocate_painter(Vec2::new(width, height), egui::Sense::hover());
        let rect = response.rect;

        let max_cn0 = 55.0_f32;
        let min_cn0 = 15.0_f32;
        let bar_width = (rect.width() / visible.len() as f32).min(25.0);

        for (i, status) in visible.iter().enumerate() {
            let cn0 = (status.cn0_dbhz as f32).clamp(min_cn0, max_cn0);
            let frac = (cn0 - min_cn0) / (max_cn0 - min_cn0);
            let bar_height = frac * (height - 30.0);

            let x = rect.min.x + i as f32 * bar_width + 2.0;
            let bar_rect = egui::Rect::from_min_max(
                Pos2::new(x, rect.max.y - bar_height - 15.0),
                Pos2::new(x + bar_width - 4.0, rect.max.y - 15.0),
            );

            let color = if cn0 > 35.0 { Color32::GREEN }
                else if cn0 > 25.0 { Color32::YELLOW }
                else { Color32::RED };

            painter.rect_filled(bar_rect, 2.0, color);

            // PRN label
            let font = egui::FontId::proportional(9.0);
            painter.text(
                Pos2::new(x + bar_width / 2.0 - 2.0, rect.max.y - 2.0),
                egui::Align2::CENTER_BOTTOM,
                format!("{}", status.prn),
                font.clone(),
                Color32::WHITE,
            );

            // C/N0 value
            painter.text(
                Pos2::new(x + bar_width / 2.0 - 2.0, rect.max.y - bar_height - 18.0),
                egui::Align2::CENTER_BOTTOM,
                format!("{:.0}", status.cn0_dbhz),
                font,
                Color32::WHITE,
            );
        }

        let title_font = egui::FontId::proportional(11.0);
        painter.text(
            Pos2::new(rect.center().x, rect.min.y + 2.0),
            egui::Align2::CENTER_TOP,
            "C/N0 (dB-Hz)",
            title_font,
            Color32::WHITE,
        );
    }

    fn render_iq_waveform(&self, ui: &mut Ui) {
        if self.iq_samples.is_empty() { return; }

        let width = ui.available_width().min(600.0);
        let height = 120.0;
        let (response, painter) = ui.allocate_painter(Vec2::new(width, height), egui::Sense::hover());
        let rect = response.rect;

        let samples = &self.iq_samples;
        let n = samples.len().min(2000);
        let step = samples.len() / n;

        // Find max amplitude for scaling
        let max_amp = samples.iter()
            .map(|s| s.re.abs().max(s.im.abs()))
            .fold(0.0_f64, f64::max)
            .max(1e-15);

        // Draw I and Q
        let mid_y = rect.center().y;
        let half_h = (height - 20.0) / 2.0;

        let mut prev_i = None;
        let mut prev_q = None;

        for i in 0..n {
            let idx = i * step;
            let x = rect.min.x + (i as f32 / n as f32) * width;

            let yi = mid_y - (samples[idx].re / max_amp) as f32 * half_h;
            let yq = mid_y - (samples[idx].im / max_amp) as f32 * half_h;

            if let Some(prev) = prev_i {
                painter.line_segment([prev, Pos2::new(x, yi)], Stroke::new(1.0, Color32::from_rgb(100, 180, 255)));
            }
            if let Some(prev) = prev_q {
                painter.line_segment([prev, Pos2::new(x, yq)], Stroke::new(1.0, Color32::from_rgb(255, 100, 100)));
            }

            prev_i = Some(Pos2::new(x, yi));
            prev_q = Some(Pos2::new(x, yq));
        }

        // Center line
        painter.line_segment(
            [Pos2::new(rect.min.x, mid_y), Pos2::new(rect.max.x, mid_y)],
            Stroke::new(0.5, Color32::DARK_GRAY),
        );

        // Legend
        let font = egui::FontId::proportional(10.0);
        painter.text(Pos2::new(rect.min.x + 5.0, rect.min.y + 2.0), egui::Align2::LEFT_TOP,
            "I (blue)", font.clone(), Color32::from_rgb(100, 180, 255));
        painter.text(Pos2::new(rect.min.x + 60.0, rect.min.y + 2.0), egui::Align2::LEFT_TOP,
            "Q (red)", font.clone(), Color32::from_rgb(255, 100, 100));

        let title_font = egui::FontId::proportional(11.0);
        painter.text(
            Pos2::new(rect.center().x, rect.min.y + 2.0),
            egui::Align2::CENTER_TOP,
            format!("Composite IQ ({} samples)", self.num_samples),
            title_font,
            Color32::WHITE,
        );
    }
}
