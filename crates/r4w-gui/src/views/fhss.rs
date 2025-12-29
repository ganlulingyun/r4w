//! FHSS Lab View - Interactive Frequency Hopping Spread Spectrum demonstration
//!
//! This view provides:
//! - Parameter controls for FHSS configuration
//! - Spectrogram visualization showing frequency hops
//! - Anti-jamming demonstration
//! - Statistics and metrics display

use egui::{Color32, Ui, RichText};
use r4w_core::waveform::fhss::{FHSS, FhssConfig, FhssSpectrogramData, HopModulation, HopPattern};
use r4w_core::waveform::fhss_antijam::{AntiJamDemo, AntiJamResult, JammerType};
use r4w_core::waveform::{CommonParams, Waveform};
use r4w_core::types::IQSample;

/// FHSS Lab View state
pub struct FhssView {
    // FHSS Configuration
    num_channels: usize,
    channel_spacing_khz: f64,
    hop_rate: f64,
    symbols_per_hop: usize,
    symbol_rate: f64,
    hop_pattern: HopPattern,
    hop_modulation: HopModulationChoice,
    sample_rate: f64,

    // Anti-Jam Demo
    jammer_type: JammerChoice,
    jammer_power: f64,
    jammer_freq_channel: usize,
    snr_db: f64,
    antijam_result: Option<AntiJamResult>,

    // Visualization
    show_spectrogram: bool,
    spectrogram_data: Option<FhssSpectrogramData>,
    fft_size: usize,

    // Generated signal
    tx_bits: Vec<u8>,
    samples: Vec<IQSample>,

    // Current FHSS instance
    fhss: Option<FHSS>,
}

/// Simplified hop modulation choice for UI
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HopModulationChoice {
    Bfsk,
    Bpsk,
    Qpsk,
}

impl HopModulationChoice {
    fn to_hop_modulation(&self) -> HopModulation {
        match self {
            Self::Bfsk => HopModulation::Bfsk { deviation: 5000.0 },
            Self::Bpsk => HopModulation::Bpsk,
            Self::Qpsk => HopModulation::Qpsk,
        }
    }

    fn name(&self) -> &'static str {
        match self {
            Self::Bfsk => "BFSK",
            Self::Bpsk => "BPSK",
            Self::Qpsk => "QPSK",
        }
    }
}

/// Jammer type choice for UI
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JammerChoice {
    Narrowband,
    Sweep,
    Follower,
    Barrage,
}

impl JammerChoice {
    fn name(&self) -> &'static str {
        match self {
            Self::Narrowband => "Narrowband",
            Self::Sweep => "Sweep",
            Self::Follower => "Follower",
            Self::Barrage => "Barrage",
        }
    }

    fn description(&self) -> &'static str {
        match self {
            Self::Narrowband => "Single frequency jammer - FHSS easily defeats this",
            Self::Sweep => "Sweeping jammer - partially effective against slow hopping",
            Self::Follower => "Tracks and follows hops - limited by reaction time",
            Self::Barrage => "Spreads power across all frequencies - least efficient",
        }
    }
}

impl Default for FhssView {
    fn default() -> Self {
        Self::new()
    }
}

impl FhssView {
    pub fn new() -> Self {
        let mut view = Self {
            num_channels: 50,
            channel_spacing_khz: 25.0,
            hop_rate: 100.0,
            symbols_per_hop: 10,
            symbol_rate: 1000.0,
            hop_pattern: HopPattern::PseudoRandom,
            hop_modulation: HopModulationChoice::Bfsk,
            sample_rate: 1_000_000.0,

            jammer_type: JammerChoice::Narrowband,
            jammer_power: 10.0,
            jammer_freq_channel: 25, // Center channel
            snr_db: 20.0,
            antijam_result: None,

            show_spectrogram: true,
            spectrogram_data: None,
            fft_size: 256,

            tx_bits: (0..200).map(|i| (i % 2) as u8).collect(),
            samples: Vec::new(),
            fhss: None,
        };
        view.rebuild_fhss();
        view
    }

    /// Rebuild FHSS instance with current parameters
    fn rebuild_fhss(&mut self) {
        let common = CommonParams {
            sample_rate: self.sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };

        let config = FhssConfig {
            num_channels: self.num_channels,
            channel_spacing: self.channel_spacing_khz * 1000.0,
            hop_rate: self.hop_rate,
            symbols_per_hop: self.symbols_per_hop,
            symbol_rate: self.symbol_rate,
            hop_pattern: self.hop_pattern,
            modulation: self.hop_modulation.to_hop_modulation(),
            seed: 0x12345,
        };

        let fhss = FHSS::new(common, config);

        // Generate samples
        self.samples = fhss.modulate(&self.tx_bits);

        // Generate spectrogram
        if self.show_spectrogram && !self.samples.is_empty() {
            self.spectrogram_data = Some(fhss.generate_spectrogram(&self.samples, self.fft_size));
        }

        self.fhss = Some(fhss);
    }

    /// Run anti-jam demo with current settings
    fn run_antijam_demo(&mut self) {
        if let Some(ref fhss) = self.fhss {
            let mut demo = AntiJamDemo::with_fhss(fhss.clone());
            demo.set_snr(self.snr_db);

            // Create jammer based on selection
            let jammer = match self.jammer_type {
                JammerChoice::Narrowband => {
                    let center_channel = self.num_channels as f64 / 2.0;
                    let freq_hz = (self.jammer_freq_channel as f64 - center_channel)
                        * self.channel_spacing_khz * 1000.0;
                    JammerType::Narrowband {
                        freq_hz,
                        power: self.jammer_power,
                    }
                }
                JammerChoice::Sweep => {
                    let half_bw = fhss.total_bandwidth() / 2.0;
                    JammerType::Sweep {
                        start_freq: -half_bw,
                        end_freq: half_bw,
                        sweep_rate: fhss.total_bandwidth() * 10.0, // 10 sweeps per second
                        power: self.jammer_power,
                    }
                }
                JammerChoice::Follower => {
                    JammerType::Follower {
                        reaction_time_sec: 0.005, // 5ms reaction time
                        power: self.jammer_power,
                        bandwidth: fhss.hop_bandwidth(),
                    }
                }
                JammerChoice::Barrage => {
                    JammerType::Barrage {
                        power: self.jammer_power * self.num_channels as f64,
                    }
                }
            };

            demo.set_jammer(jammer);
            self.antijam_result = Some(demo.run(&self.tx_bits));
        }
    }

    pub fn render(&mut self, ui: &mut Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            // Header
            ui.heading("FHSS Lab - Frequency Hopping Spread Spectrum");
            ui.add_space(4.0);
            ui.label("Explore frequency hopping and its resistance to jamming");
            ui.add_space(12.0);

            // Configuration section
            ui.collapsing(RichText::new("FHSS Configuration").heading(), |ui| {
                let mut changed = false;

                egui::Grid::new("fhss_config_grid")
                    .num_columns(2)
                    .spacing([20.0, 8.0])
                    .show(ui, |ui| {
                        ui.label("Number of Channels:");
                        if ui.add(egui::Slider::new(&mut self.num_channels, 10..=100)).changed() {
                            changed = true;
                        }
                        ui.end_row();

                        ui.label("Channel Spacing (kHz):");
                        if ui.add(egui::Slider::new(&mut self.channel_spacing_khz, 5.0..=100.0)
                            .suffix(" kHz")).changed() {
                            changed = true;
                        }
                        ui.end_row();

                        ui.label("Hop Rate (hops/sec):");
                        if ui.add(egui::Slider::new(&mut self.hop_rate, 10.0..=1000.0)
                            .suffix(" hops/s")).changed() {
                            changed = true;
                        }
                        ui.end_row();

                        ui.label("Symbols per Hop:");
                        if ui.add(egui::Slider::new(&mut self.symbols_per_hop, 1..=50)).changed() {
                            changed = true;
                        }
                        ui.end_row();

                        ui.label("Symbol Rate (sym/sec):");
                        if ui.add(egui::Slider::new(&mut self.symbol_rate, 100.0..=10000.0)
                            .suffix(" sym/s")).changed() {
                            changed = true;
                        }
                        ui.end_row();

                        ui.label("Hop Pattern:");
                        egui::ComboBox::from_id_salt("hop_pattern")
                            .selected_text(match self.hop_pattern {
                                HopPattern::PseudoRandom => "Pseudo-Random (LFSR)",
                                HopPattern::Sequential => "Sequential",
                            })
                            .show_ui(ui, |ui| {
                                if ui.selectable_value(&mut self.hop_pattern, HopPattern::PseudoRandom, "Pseudo-Random (LFSR)").changed() {
                                    changed = true;
                                }
                                if ui.selectable_value(&mut self.hop_pattern, HopPattern::Sequential, "Sequential").changed() {
                                    changed = true;
                                }
                            });
                        ui.end_row();

                        ui.label("Hop Modulation:");
                        egui::ComboBox::from_id_salt("hop_mod")
                            .selected_text(self.hop_modulation.name())
                            .show_ui(ui, |ui| {
                                for choice in [HopModulationChoice::Bfsk, HopModulationChoice::Bpsk, HopModulationChoice::Qpsk] {
                                    if ui.selectable_value(&mut self.hop_modulation, choice, choice.name()).changed() {
                                        changed = true;
                                    }
                                }
                            });
                        ui.end_row();
                    });

                if changed {
                    self.rebuild_fhss();
                }
            });

            ui.add_space(12.0);

            // Statistics
            if let Some(ref fhss) = self.fhss {
                ui.collapsing(RichText::new("FHSS Statistics").heading(), |ui| {
                    egui::Grid::new("fhss_stats")
                        .num_columns(2)
                        .spacing([20.0, 4.0])
                        .show(ui, |ui| {
                            ui.label("Processing Gain:");
                            ui.label(format!("{:.1} dB", fhss.processing_gain_db()));
                            ui.end_row();

                            ui.label("Total Bandwidth:");
                            ui.label(format!("{:.1} kHz", fhss.total_bandwidth() / 1000.0));
                            ui.end_row();

                            ui.label("Hop Bandwidth:");
                            ui.label(format!("{:.1} kHz", fhss.hop_bandwidth() / 1000.0));
                            ui.end_row();

                            ui.label("Dwell Time:");
                            ui.label(format!("{:.1} ms", fhss.dwell_time() * 1000.0));
                            ui.end_row();

                            ui.label("Data Rate:");
                            ui.label(format!("{:.0} bps", fhss.data_rate()));
                            ui.end_row();

                            ui.label("Samples per Hop:");
                            ui.label(format!("{}", fhss.samples_per_hop()));
                            ui.end_row();
                        });
                });
            }

            ui.add_space(12.0);

            // Spectrogram visualization
            ui.collapsing(RichText::new("Spectrogram Visualization").heading(), |ui| {
                ui.checkbox(&mut self.show_spectrogram, "Show Spectrogram");

                ui.horizontal(|ui| {
                    ui.label("FFT Size:");
                    egui::ComboBox::from_id_salt("fft_size")
                        .selected_text(format!("{}", self.fft_size))
                        .show_ui(ui, |ui| {
                            for size in [64, 128, 256, 512, 1024] {
                                if ui.selectable_value(&mut self.fft_size, size, format!("{}", size)).changed() {
                                    self.rebuild_fhss();
                                }
                            }
                        });
                });

                if self.show_spectrogram {
                    if let Some(ref spec_data) = self.spectrogram_data {
                        self.render_spectrogram(ui, spec_data);
                    }
                }
            });

            ui.add_space(12.0);

            // Anti-Jam Demo
            ui.collapsing(RichText::new("Anti-Jamming Demo").heading(), |ui| {
                ui.label("Demonstrate how FHSS defeats different jamming strategies");
                ui.add_space(8.0);

                egui::Grid::new("antijam_config")
                    .num_columns(2)
                    .spacing([20.0, 8.0])
                    .show(ui, |ui| {
                        ui.label("Jammer Type:");
                        egui::ComboBox::from_id_salt("jammer_type")
                            .selected_text(self.jammer_type.name())
                            .show_ui(ui, |ui| {
                                for choice in [JammerChoice::Narrowband, JammerChoice::Sweep, JammerChoice::Follower, JammerChoice::Barrage] {
                                    ui.selectable_value(&mut self.jammer_type, choice, choice.name());
                                }
                            });
                        ui.end_row();

                        ui.label("Jammer Power:");
                        ui.add(egui::Slider::new(&mut self.jammer_power, 1.0..=100.0)
                            .logarithmic(true)
                            .suffix("x signal"));
                        ui.end_row();

                        if self.jammer_type == JammerChoice::Narrowband {
                            ui.label("Jammer Channel:");
                            ui.add(egui::Slider::new(&mut self.jammer_freq_channel, 0..=self.num_channels - 1));
                            ui.end_row();
                        }

                        ui.label("SNR (dB):");
                        ui.add(egui::Slider::new(&mut self.snr_db, 0.0..=40.0).suffix(" dB"));
                        ui.end_row();
                    });

                ui.add_space(4.0);
                ui.label(RichText::new(self.jammer_type.description()).small().italics());
                ui.add_space(8.0);

                if ui.button("Run Anti-Jam Demo").clicked() {
                    self.run_antijam_demo();
                }

                // Show results
                if let Some(ref result) = self.antijam_result {
                    ui.add_space(12.0);
                    ui.separator();
                    ui.heading("Results");

                    egui::Grid::new("antijam_results")
                        .num_columns(2)
                        .spacing([20.0, 4.0])
                        .show(ui, |ui| {
                            ui.label("Clean BER:");
                            let clean_color = if result.clean_ber < 0.01 {
                                Color32::from_rgb(100, 255, 100)
                            } else {
                                Color32::from_rgb(255, 200, 100)
                            };
                            ui.colored_label(clean_color, format!("{:.2e}", result.clean_ber));
                            ui.end_row();

                            ui.label("Jammed BER:");
                            let jammed_color = if result.jammed_ber < 0.01 {
                                Color32::from_rgb(100, 255, 100)
                            } else if result.jammed_ber < 0.1 {
                                Color32::from_rgb(255, 200, 100)
                            } else {
                                Color32::from_rgb(255, 100, 100)
                            };
                            ui.colored_label(jammed_color, format!("{:.2e}", result.jammed_ber));
                            ui.end_row();

                            ui.label("Hops Affected:");
                            ui.label(format!("{} / {} ({:.1}%)",
                                result.hops_affected,
                                result.total_hops,
                                result.percent_affected));
                            ui.end_row();

                            ui.label("Theoretical Gain:");
                            ui.label(format!("{:.1} dB", result.theoretical_gain_db));
                            ui.end_row();

                            ui.label("Signal-to-Jammer:");
                            ui.label(format!("{:.1} dB", result.sjr_db));
                            ui.end_row();
                        });

                    // Interpretation
                    ui.add_space(8.0);
                    let interpretation = if result.jammed_ber < 0.01 {
                        ("FHSS successfully defeated the jammer!", Color32::from_rgb(100, 255, 100))
                    } else if result.jammed_ber < result.clean_ber * 10.0 {
                        ("FHSS provided significant protection", Color32::from_rgb(200, 255, 100))
                    } else {
                        ("Jammer caused noticeable degradation", Color32::from_rgb(255, 150, 100))
                    };
                    ui.colored_label(interpretation.1, RichText::new(interpretation.0).strong());
                }
            });

            ui.add_space(12.0);

            // Educational content
            ui.collapsing(RichText::new("How FHSS Works").heading(), |ui| {
                ui.label("FHSS spreads the signal across many frequencies by 'hopping' according to a pseudo-random sequence.");
                ui.add_space(8.0);

                ui.label(RichText::new("Key Benefits:").strong());
                ui.indent("fhss_benefits", |ui| {
                    ui.label("• Interference Avoidance: Signal quickly moves away from interference");
                    ui.label("• Anti-Jam: Jammer must spread power across entire bandwidth");
                    ui.label("• LPD/LPI: Low Probability of Detection/Intercept");
                    ui.label("• Multiple Access: Different hop patterns for different users");
                });

                ui.add_space(8.0);
                ui.label(RichText::new("Processing Gain:").strong());
                ui.label("The processing gain is approximately 10*log10(N) where N is the number of channels.");
                ui.label(format!("With {} channels, the gain is {:.1} dB",
                    self.num_channels,
                    10.0 * (self.num_channels as f64).log10()));
            });
        });
    }

    /// Render spectrogram visualization
    fn render_spectrogram(&self, ui: &mut Ui, spec_data: &FhssSpectrogramData) {
        let available_width = ui.available_width().min(800.0);
        let plot_height = 300.0;

        let (response, painter) = ui.allocate_painter(
            egui::vec2(available_width, plot_height),
            egui::Sense::hover(),
        );
        let rect = response.rect;

        // Background
        painter.rect_filled(rect, 0.0, Color32::from_gray(20));

        if spec_data.power_grid.is_empty() || spec_data.power_grid[0].is_empty() {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "No spectrogram data",
                egui::FontId::default(),
                Color32::GRAY,
            );
            return;
        }

        let num_time_bins = spec_data.power_grid.len();
        let num_freq_bins = spec_data.power_grid[0].len();

        // Find power range for coloring
        let mut min_power = f64::MAX;
        let mut max_power = f64::MIN;
        for row in &spec_data.power_grid {
            for &p in row {
                if p > f64::MIN && p < f64::MAX {
                    min_power = min_power.min(p);
                    max_power = max_power.max(p);
                }
            }
        }

        let power_range = (max_power - min_power).max(1.0);

        // Draw spectrogram pixels
        let cell_width = rect.width() / num_time_bins as f32;
        let cell_height = rect.height() / num_freq_bins as f32;

        for (t_idx, row) in spec_data.power_grid.iter().enumerate() {
            for (f_idx, &power) in row.iter().enumerate() {
                let normalized = ((power - min_power) / power_range).clamp(0.0, 1.0);
                let color = self.power_to_color(normalized);

                let x = rect.left() + t_idx as f32 * cell_width;
                let y = rect.top() + (num_freq_bins - 1 - f_idx) as f32 * cell_height;

                painter.rect_filled(
                    egui::Rect::from_min_size(egui::pos2(x, y), egui::vec2(cell_width + 1.0, cell_height + 1.0)),
                    0.0,
                    color,
                );
            }
        }

        // Draw hop markers
        if !spec_data.hop_markers.is_empty() && !spec_data.time_axis.is_empty() {
            let time_range = spec_data.time_axis.last().unwrap_or(&1.0) - spec_data.time_axis.first().unwrap_or(&0.0);
            let freq_range = spec_data.total_bandwidth;

            for (time, freq) in &spec_data.hop_markers {
                let x = rect.left() + (time / time_range.max(0.001)) as f32 * rect.width();
                let y_norm = (freq + freq_range / 2.0) / freq_range;
                let y = rect.bottom() - y_norm as f32 * rect.height();

                painter.circle_stroke(
                    egui::pos2(x.clamp(rect.left(), rect.right()), y.clamp(rect.top(), rect.bottom())),
                    3.0,
                    egui::Stroke::new(1.5, Color32::WHITE),
                );
            }
        }

        // Labels
        painter.text(
            egui::pos2(rect.center().x, rect.bottom() + 15.0),
            egui::Align2::CENTER_TOP,
            "Time",
            egui::FontId::default(),
            Color32::GRAY,
        );

        ui.add_space(20.0);
        ui.horizontal(|ui| {
            ui.label("Color: ");
            ui.colored_label(Color32::from_rgb(0, 0, 50), "Low");
            ui.label(" to ");
            ui.colored_label(Color32::from_rgb(255, 255, 0), "High");
            ui.label(" power | ");
            ui.label("White circles: Hop positions");
        });
    }

    /// Convert normalized power (0-1) to color
    fn power_to_color(&self, normalized: f64) -> Color32 {
        // Blue -> Cyan -> Green -> Yellow -> Red colormap
        let n = normalized as f32;
        if n < 0.25 {
            let t = n / 0.25;
            Color32::from_rgb(0, (t * 128.0) as u8, (50.0 + t * 205.0) as u8)
        } else if n < 0.5 {
            let t = (n - 0.25) / 0.25;
            Color32::from_rgb(0, (128.0 + t * 127.0) as u8, (255.0 - t * 128.0) as u8)
        } else if n < 0.75 {
            let t = (n - 0.5) / 0.25;
            Color32::from_rgb((t * 255.0) as u8, 255, (127.0 - t * 127.0) as u8)
        } else {
            let t = (n - 0.75) / 0.25;
            Color32::from_rgb(255, (255.0 - t * 128.0) as u8, 0)
        }
    }
}
