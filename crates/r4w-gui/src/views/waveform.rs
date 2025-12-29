//! Waveform Explorer view - Interactive waveform development kit

use egui::{Color32, Ui};
use r4w_core::types::IQSample;
use r4w_core::waveform::{Waveform, WaveformFactory, VisualizationData};
use r4w_sim::ChannelModel;
use rand::Rng;
use rand::rngs::StdRng;
use rand::SeedableRng;

/// External parameters passed from the sidebar
#[derive(Clone)]
pub struct WaveformParams {
    /// Selected waveform name
    pub waveform_name: String,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Test bits as a string
    pub test_bits: String,
    /// SNR in dB
    pub snr_db: f32,
    /// Channel model
    pub channel_model: ChannelModel,
    /// Pre-generated samples from sidebar (if any)
    pub samples: Option<Vec<IQSample>>,
    /// BER from sidebar demodulation
    pub ber: Option<f64>,
}

/// Add AWGN (Additive White Gaussian Noise) to samples
fn add_awgn_to_samples(rng: &mut StdRng, samples: &[IQSample], snr_db: f64) -> Vec<IQSample> {
    if samples.is_empty() {
        return Vec::new();
    }

    // Calculate signal power
    let signal_power: f64 = samples.iter()
        .map(|s| s.re * s.re + s.im * s.im)
        .sum::<f64>() / samples.len() as f64;

    // Calculate noise power from SNR
    // SNR = 10 * log10(signal_power / noise_power)
    // noise_power = signal_power / 10^(SNR/10)
    let snr_linear = 10.0_f64.powf(snr_db / 10.0);
    let noise_power = signal_power / snr_linear;
    let noise_std = (noise_power / 2.0).sqrt(); // Divide by 2 for I and Q components

    // Generate noisy samples using Box-Muller transform
    samples.iter().map(|s| {
        let u1: f64 = rng.gen::<f64>().max(1e-10);
        let u2: f64 = rng.gen::<f64>();
        let mag = (-2.0_f64 * u1.ln()).sqrt();
        let phase = 2.0 * std::f64::consts::PI * u2;
        let noise_i = mag * phase.cos() * noise_std;
        let noise_q = mag * phase.sin() * noise_std;
        IQSample::new(s.re + noise_i, s.im + noise_q)
    }).collect()
}

/// A test bit sequence with its own color
#[derive(Clone)]
struct TestSequence {
    /// The bit pattern as a string for editing
    bits_str: String,
    /// Parsed bits
    bits: Vec<u8>,
    /// Display color for this sequence
    color: Color32,
    /// Generated clean samples
    samples: Vec<IQSample>,
    /// Generated noisy samples
    noisy_samples: Vec<IQSample>,
    /// Demodulated bits from noisy signal
    demod_bits: Vec<u8>,
    /// Bit errors count
    bit_errors: usize,
}

impl TestSequence {
    fn new(bits_str: &str, color: Color32) -> Self {
        let bits: Vec<u8> = bits_str
            .chars()
            .filter_map(|c| match c {
                '0' => Some(0u8),
                '1' => Some(1u8),
                _ => None,
            })
            .collect();
        Self {
            bits_str: bits_str.to_string(),
            bits,
            color,
            samples: Vec::new(),
            noisy_samples: Vec::new(),
            demod_bits: Vec::new(),
            bit_errors: 0,
        }
    }

    fn update_bits(&mut self) {
        self.bits = self.bits_str
            .chars()
            .filter_map(|c| match c {
                '0' => Some(0u8),
                '1' => Some(1u8),
                _ => None,
            })
            .collect();
    }
}

/// Predefined colors for sequences
const SEQUENCE_COLORS: [Color32; 8] = [
    Color32::from_rgb(100, 200, 100),  // Green
    Color32::from_rgb(200, 100, 100),  // Red
    Color32::from_rgb(100, 100, 255),  // Blue
    Color32::from_rgb(255, 200, 50),   // Yellow
    Color32::from_rgb(200, 100, 200),  // Purple
    Color32::from_rgb(100, 200, 200),  // Cyan
    Color32::from_rgb(255, 150, 100),  // Orange
    Color32::from_rgb(150, 255, 150),  // Light green
];

/// View for exploring different waveform types
pub struct WaveformView {
    /// Currently selected waveform type
    selected_waveform: String,
    /// Sample rate for waveforms
    sample_rate: f64,
    /// Multiple test sequences
    sequences: Vec<TestSequence>,
    /// Current visualization data (constellation from first sequence)
    viz_data: Option<VisualizationData>,
    /// Current waveform instance
    current_waveform: Option<Box<dyn Waveform>>,
    /// Show I component
    show_i: bool,
    /// Show Q component
    show_q: bool,
    /// Show constellation
    show_constellation: bool,
    /// Use separate graphs for each sequence
    separate_graphs: bool,
    /// Enable noise simulation
    noise_enabled: bool,
    /// SNR in dB
    snr_db: f32,
    /// Show noisy signal
    show_noisy: bool,
    /// Show clean signal when noise is enabled
    show_clean: bool,
}

impl WaveformView {
    pub fn new() -> Self {
        let mut view = Self {
            selected_waveform: "BPSK".to_string(),
            sample_rate: 125_000.0,
            sequences: vec![
                TestSequence::new("10110010", SEQUENCE_COLORS[0]),
            ],
            viz_data: None,
            current_waveform: None,
            show_i: true,
            show_q: true,
            show_constellation: true,
            separate_graphs: false,
            noise_enabled: false,
            snr_db: 10.0,
            show_noisy: true,
            show_clean: true,
        };
        view.update_waveform();
        view
    }

    /// Update noisy samples and demodulation
    fn update_noise(&mut self) {
        // Reseed RNG for reproducible noise at same SNR
        let mut rng = StdRng::seed_from_u64(42);
        let snr_db = self.snr_db as f64;
        let noise_enabled = self.noise_enabled;

        // First pass: generate noisy samples for all sequences
        let noisy_data: Vec<Vec<IQSample>> = if noise_enabled {
            self.sequences.iter()
                .map(|seq| {
                    if !seq.samples.is_empty() {
                        add_awgn_to_samples(&mut rng, &seq.samples, snr_db)
                    } else {
                        Vec::new()
                    }
                })
                .collect()
        } else {
            vec![Vec::new(); self.sequences.len()]
        };

        // Second pass: demodulate and calculate errors
        for (idx, noisy_samples) in noisy_data.into_iter().enumerate() {
            let seq = &mut self.sequences[idx];

            if noise_enabled && !noisy_samples.is_empty() {
                // Demodulate noisy signal
                if let Some(ref wf) = self.current_waveform {
                    let result = wf.demodulate(&noisy_samples);
                    seq.demod_bits = result.bits;
                }

                // Calculate bit errors
                seq.bit_errors = seq.bits.iter()
                    .zip(seq.demod_bits.iter())
                    .filter(|(&a, &b)| a != b)
                    .count();

                seq.noisy_samples = noisy_samples;
            } else {
                seq.noisy_samples.clear();
                seq.demod_bits.clear();
                seq.bit_errors = 0;
            }
        }
    }

    /// Update the waveform instance and visualization
    fn update_waveform(&mut self) {
        self.current_waveform = WaveformFactory::create(&self.selected_waveform, self.sample_rate);

        if let Some(ref waveform) = self.current_waveform {
            // Update samples for each sequence
            for seq in &mut self.sequences {
                seq.samples = waveform.modulate(&seq.bits);
            }

            // Get visualization data from first sequence for constellation
            if !self.sequences.is_empty() {
                self.viz_data = Some(waveform.get_visualization(&self.sequences[0].bits));
            }
        }

        // Update noisy samples
        self.update_noise();
    }

    /// Add a new test sequence
    fn add_sequence(&mut self) {
        let color_idx = self.sequences.len() % SEQUENCE_COLORS.len();
        let default_bits = match self.sequences.len() {
            1 => "01001101",
            2 => "11110000",
            3 => "00001111",
            _ => "10101010",
        };
        self.sequences.push(TestSequence::new(default_bits, SEQUENCE_COLORS[color_idx]));
        self.update_waveform();
    }

    /// Remove a test sequence by index
    fn remove_sequence(&mut self, idx: usize) {
        if self.sequences.len() > 1 && idx < self.sequences.len() {
            self.sequences.remove(idx);
            self.update_waveform();
        }
    }

    /// Draw a time domain plot for given samples
    fn draw_time_plot(&self, painter: &egui::Painter, rect: egui::Rect,
                       samples: &[IQSample], i_color: Color32, q_color: Color32, max_amp: f64,
                       stroke_width: f32) {
        let center_y = rect.center().y;
        let max_points = (rect.width() as usize).min(samples.len());
        if max_points < 2 || samples.is_empty() {
            return;
        }
        let step = samples.len() / max_points.max(1);

        // Draw I component
        if self.show_i {
            let mut i_points: Vec<egui::Pos2> = Vec::new();
            for (idx, sample) in samples.iter().step_by(step.max(1)).enumerate() {
                let x = rect.left() + rect.width() * idx as f32 / max_points as f32;
                let y = center_y - (sample.re as f32 / max_amp as f32) * (rect.height() / 2.0 - 10.0);
                i_points.push(egui::pos2(x, y));
            }
            if i_points.len() >= 2 {
                for points in i_points.windows(2) {
                    painter.line_segment(
                        [points[0], points[1]],
                        egui::Stroke::new(stroke_width, i_color),
                    );
                }
            }
        }

        // Draw Q component
        if self.show_q {
            let mut q_points: Vec<egui::Pos2> = Vec::new();
            for (idx, sample) in samples.iter().step_by(step.max(1)).enumerate() {
                let x = rect.left() + rect.width() * idx as f32 / max_points as f32;
                let y = center_y - (sample.im as f32 / max_amp as f32) * (rect.height() / 2.0 - 10.0);
                q_points.push(egui::pos2(x, y));
            }
            if q_points.len() >= 2 {
                for points in q_points.windows(2) {
                    painter.line_segment(
                        [points[0], points[1]],
                        egui::Stroke::new(stroke_width, q_color.linear_multiply(0.6)),
                    );
                }
            }
        }
    }

    /// Draw plot background and grid
    fn draw_plot_background(&self, painter: &egui::Painter, rect: egui::Rect) {
        painter.rect_filled(rect, 0.0, Color32::from_gray(20));

        // Draw grid
        let grid_color = Color32::from_gray(40);
        for i in 0..=4 {
            let y = rect.top() + rect.height() * i as f32 / 4.0;
            painter.line_segment(
                [egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)],
                egui::Stroke::new(1.0, grid_color),
            );
        }

        // Center line
        let center_y = rect.center().y;
        painter.line_segment(
            [egui::pos2(rect.left(), center_y), egui::pos2(rect.right(), center_y)],
            egui::Stroke::new(1.0, Color32::from_gray(60)),
        );
    }

    /// Calculate total BER across all sequences
    fn calculate_total_ber(&self) -> (usize, usize, f64) {
        let mut total_errors = 0;
        let mut total_bits = 0;
        for seq in &self.sequences {
            total_errors += seq.bit_errors;
            total_bits += seq.bits.len().min(seq.demod_bits.len());
        }
        let ber = if total_bits > 0 {
            total_errors as f64 / total_bits as f64
        } else {
            0.0
        };
        (total_errors, total_bits, ber)
    }

    /// Sync with external parameters from sidebar
    pub fn sync_with_params(&mut self, params: &WaveformParams) {
        let mut needs_update = false;

        // Sync waveform selection
        if self.selected_waveform != params.waveform_name {
            self.selected_waveform = params.waveform_name.clone();
            needs_update = true;
        }

        // Sync sample rate
        if (self.sample_rate - params.sample_rate).abs() > 0.1 {
            self.sample_rate = params.sample_rate;
            needs_update = true;
        }

        // NOTE: Removed one-way sync from sidebar to WaveformView's sequences
        // This was causing the textbox to be uneditable (user's typing was overwritten each frame)
        // The WaveformView now manages its own sequences independently

        // Sync noise settings
        if params.channel_model != ChannelModel::Ideal {
            if !self.noise_enabled {
                self.noise_enabled = true;
                needs_update = true;
            }
            if (self.snr_db - params.snr_db).abs() > 0.1 {
                self.snr_db = params.snr_db;
                needs_update = true;
            }
        } else if self.noise_enabled {
            self.noise_enabled = false;
            needs_update = true;
        }

        // If sidebar has pre-generated samples, use them for first sequence
        if let Some(ref samples) = params.samples {
            if !samples.is_empty() && !self.sequences.is_empty() {
                // Only update if different (compare length as quick check)
                if self.sequences[0].samples.len() != samples.len() {
                    self.sequences[0].samples = samples.clone();
                    // Also update visualization
                    if let Some(ref wf) = self.current_waveform {
                        self.viz_data = Some(wf.get_visualization(&self.sequences[0].bits));
                    }
                    needs_update = true;
                }
            }
        }

        if needs_update {
            self.update_waveform();
        }
    }

    pub fn render(&mut self, ui: &mut Ui) {
        self.render_with_params(ui, None)
    }

    pub fn render_with_params(&mut self, ui: &mut Ui, external_params: Option<&WaveformParams>) {
        // Sync with external params if provided
        if let Some(params) = external_params {
            self.sync_with_params(params);
        }
        egui::ScrollArea::vertical().show(ui, |ui| {
            // Header with waveform info
            if let Some(ref waveform) = self.current_waveform {
                let info = waveform.info();
                ui.horizontal(|ui| {
                    ui.heading(info.full_name);
                    ui.label(format!("({})", info.name));
                });
                ui.label(info.description);

                // Characteristics
                ui.add_space(8.0);
                ui.horizontal_wrapped(|ui| {
                    for char in info.characteristics {
                        ui.label(format!("• {}", char));
                    }
                });

                // Key stats
                ui.add_space(8.0);
                egui::Grid::new("waveform_stats")
                    .num_columns(4)
                    .spacing([20.0, 4.0])
                    .show(ui, |ui| {
                        ui.label("Complexity:");
                        ui.label(format!("{}/5", info.complexity));
                        ui.label("Bits/Symbol:");
                        ui.label(if info.bits_per_symbol > 0 {
                            format!("{}", info.bits_per_symbol)
                        } else {
                            "N/A".to_string()
                        });
                        ui.end_row();
                    });

                // History and Modern Usage (collapsible)
                ui.add_space(8.0);
                egui::CollapsingHeader::new("📜 History & Modern Usage")
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.add_space(4.0);
                        ui.label(egui::RichText::new("History").strong());
                        ui.label(info.history);
                        ui.add_space(8.0);
                        ui.label(egui::RichText::new("Modern Usage").strong());
                        ui.label(info.modern_usage);
                    });
            }

            ui.add_space(12.0);
            ui.separator();

            // Track if waveform changed (from params sync)
            let mut changed = false;

            // Test sequences section
            ui.horizontal(|ui| {
                ui.heading("Test Bit Sequences");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.button("+ Add Sequence").clicked() {
                        self.add_sequence();
                    }
                });
            });
            ui.add_space(8.0);

            // Render each sequence input
            let mut to_remove: Option<usize> = None;
            let num_sequences = self.sequences.len();
            for idx in 0..num_sequences {
                let seq = &mut self.sequences[idx];
                let seq_color = seq.color;
                let seq_bits_len = seq.bits.len();

                ui.horizontal(|ui| {
                    // Color indicator
                    let (rect, _) = ui.allocate_exact_size(egui::vec2(16.0, 16.0), egui::Sense::hover());
                    ui.painter().rect_filled(rect, 3.0, seq_color);

                    ui.label(format!("#{}:", idx + 1));

                    let response = ui.add(
                        egui::TextEdit::singleline(&mut seq.bits_str)
                            .desired_width(200.0)
                            .hint_text("Enter bits (0s and 1s)")
                    );
                    if response.changed() {
                        seq.update_bits();
                        changed = true;
                    }

                    ui.label(format!("({} bits)", seq_bits_len));

                    // Remove button (only if more than one sequence)
                    if num_sequences > 1 {
                        if ui.button("X").clicked() {
                            to_remove = Some(idx);
                        }
                    }
                });
            }

            // Handle removal after iteration
            if let Some(idx) = to_remove {
                self.remove_sequence(idx);
                changed = true;
            }

            if changed {
                self.update_waveform();
            }

            ui.add_space(12.0);
            ui.separator();

            // Noise simulation section
            ui.heading("Channel Simulation");
            ui.add_space(8.0);

            let mut noise_changed = false;

            ui.horizontal(|ui| {
                if ui.checkbox(&mut self.noise_enabled, "Enable AWGN Noise").changed() {
                    noise_changed = true;
                }
            });

            if self.noise_enabled {
                ui.horizontal(|ui| {
                    ui.label("SNR (dB):");
                    let slider = egui::Slider::new(&mut self.snr_db, -5.0..=30.0)
                        .suffix(" dB")
                        .clamping(egui::SliderClamping::Always);
                    if ui.add(slider).changed() {
                        noise_changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.show_clean, "Show Clean Signal");
                    ui.checkbox(&mut self.show_noisy, "Show Noisy Signal");
                });

                // Show BER statistics
                let (total_errors, total_bits, ber) = self.calculate_total_ber();
                ui.add_space(8.0);

                let ber_color = if ber == 0.0 {
                    Color32::from_rgb(100, 255, 100)
                } else if ber < 0.01 {
                    Color32::from_rgb(200, 255, 100)
                } else if ber < 0.1 {
                    Color32::from_rgb(255, 200, 100)
                } else {
                    Color32::from_rgb(255, 100, 100)
                };

                egui::Frame::none()
                    .fill(Color32::from_gray(30))
                    .inner_margin(8.0)
                    .rounding(4.0)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Performance:");
                            ui.colored_label(ber_color, format!("BER = {:.2e}", ber));
                            ui.label(format!("({} errors / {} bits)", total_errors, total_bits));
                        });

                        // Per-sequence breakdown
                        if self.sequences.len() > 1 {
                            ui.add_space(4.0);
                            for (idx, seq) in self.sequences.iter().enumerate() {
                                let seq_ber = if !seq.demod_bits.is_empty() {
                                    seq.bit_errors as f64 / seq.bits.len().min(seq.demod_bits.len()) as f64
                                } else {
                                    0.0
                                };
                                ui.horizontal(|ui| {
                                    let (rect, _) = ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
                                    ui.painter().rect_filled(rect, 2.0, seq.color);
                                    ui.label(format!("Seq #{}: {} errors ({:.1}%)", idx + 1, seq.bit_errors, seq_ber * 100.0));
                                });
                            }
                        }
                    });
            }

            if noise_changed {
                self.update_noise();
            }

            ui.add_space(12.0);
            ui.separator();

            // Visualization controls
            ui.heading("Visualization");
            ui.add_space(8.0);

            ui.horizontal(|ui| {
                ui.checkbox(&mut self.show_i, "I Component");
                ui.checkbox(&mut self.show_q, "Q Component");
                ui.checkbox(&mut self.show_constellation, "Constellation");
                ui.checkbox(&mut self.separate_graphs, "Separate Graphs");
            });

            ui.add_space(12.0);

            // Calculate global max amplitude for consistent scaling
            let global_max_amp = {
                let clean_max = self.sequences.iter()
                    .flat_map(|s| s.samples.iter())
                    .map(|s| s.re.abs().max(s.im.abs()))
                    .fold(0.1f64, f64::max);

                let noisy_max = if self.noise_enabled {
                    self.sequences.iter()
                        .flat_map(|s| s.noisy_samples.iter())
                        .map(|s| s.re.abs().max(s.im.abs()))
                        .fold(0.1f64, f64::max)
                } else {
                    0.1
                };

                clean_max.max(noisy_max)
            };

            // Time domain plot(s)
            let has_samples = self.sequences.iter().any(|s| !s.samples.is_empty());

            if has_samples {
                ui.heading("Time Domain");

                if let Some(ref viz) = self.viz_data {
                    ui.label(&viz.description);
                }

                let available_width = ui.available_width();

                if self.separate_graphs {
                    // Separate graph for each sequence
                    for (idx, seq) in self.sequences.iter().enumerate() {
                        if seq.samples.is_empty() {
                            continue;
                        }

                        ui.add_space(8.0);
                        ui.horizontal(|ui| {
                            let (rect, _) = ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
                            ui.painter().rect_filled(rect, 2.0, seq.color);
                            ui.label(format!("Sequence #{}: {}", idx + 1, seq.bits_str));
                            if self.noise_enabled && !seq.demod_bits.is_empty() {
                                let ber = seq.bit_errors as f64 / seq.bits.len().min(seq.demod_bits.len()) as f64;
                                ui.label(format!("| BER: {:.2e}", ber));
                            }
                        });

                        let plot_height = 150.0;
                        let (response, painter) = ui.allocate_painter(
                            egui::vec2(available_width, plot_height),
                            egui::Sense::hover(),
                        );
                        let rect = response.rect;

                        self.draw_plot_background(&painter, rect);

                        // Draw clean signal (if enabled or no noise)
                        if !self.noise_enabled || self.show_clean {
                            let q_color = Color32::from_rgb(
                                (seq.color.r() as u16 * 6 / 10) as u8,
                                (seq.color.g() as u16 * 6 / 10) as u8,
                                (seq.color.b() as u16 * 6 / 10) as u8,
                            );
                            self.draw_time_plot(&painter, rect, &seq.samples, seq.color, q_color, global_max_amp, 2.0);
                        }

                        // Draw noisy signal
                        if self.noise_enabled && self.show_noisy && !seq.noisy_samples.is_empty() {
                            let noisy_color = Color32::from_rgb(255, 150, 150);
                            let noisy_q_color = Color32::from_rgb(150, 100, 100);
                            self.draw_time_plot(&painter, rect, &seq.noisy_samples, noisy_color, noisy_q_color, global_max_amp, 1.0);
                        }
                    }
                } else {
                    // Combined graph with all sequences overlaid
                    let plot_height = 200.0;
                    let (response, painter) = ui.allocate_painter(
                        egui::vec2(available_width, plot_height),
                        egui::Sense::hover(),
                    );
                    let rect = response.rect;

                    self.draw_plot_background(&painter, rect);

                    // Draw each sequence
                    for seq in &self.sequences {
                        if seq.samples.is_empty() {
                            continue;
                        }

                        // Draw clean signal (if enabled or no noise)
                        if !self.noise_enabled || self.show_clean {
                            let q_color = Color32::from_rgb(
                                (seq.color.r() as u16 * 6 / 10) as u8,
                                (seq.color.g() as u16 * 6 / 10) as u8,
                                (seq.color.b() as u16 * 6 / 10) as u8,
                            );
                            self.draw_time_plot(&painter, rect, &seq.samples, seq.color, q_color, global_max_amp, 2.0);
                        }

                        // Draw noisy signal
                        if self.noise_enabled && self.show_noisy && !seq.noisy_samples.is_empty() {
                            // Use a lighter/transparent version for noisy
                            let noisy_color = Color32::from_rgba_unmultiplied(
                                seq.color.r(),
                                seq.color.g(),
                                seq.color.b(),
                                128
                            );
                            let noisy_q_color = noisy_color.linear_multiply(0.6);
                            self.draw_time_plot(&painter, rect, &seq.noisy_samples, noisy_color, noisy_q_color, global_max_amp, 1.0);
                        }
                    }
                }

                // Legend
                ui.add_space(4.0);
                ui.horizontal_wrapped(|ui| {
                    for (idx, seq) in self.sequences.iter().enumerate() {
                        ui.colored_label(seq.color, format!("#{}", idx + 1));
                        ui.label(format!("({})", if seq.bits_str.len() > 10 {
                            format!("{}...", &seq.bits_str[..10])
                        } else {
                            seq.bits_str.clone()
                        }));
                        ui.add_space(10.0);
                    }
                });
                if self.noise_enabled {
                    ui.label("(Solid = Clean, Faded = Noisy)");
                } else if self.show_i && self.show_q {
                    ui.label("(Solid = I, Faded = Q)");
                }
            }

            // Constellation diagram
            if let Some(ref viz) = self.viz_data {
                if self.show_constellation && !viz.constellation.is_empty() {
                    ui.add_space(20.0);
                    ui.heading("Constellation Diagram");
                    if self.noise_enabled {
                        ui.label("Showing effect of noise on symbol positions");
                    }

                    let plot_size = 300.0f32.min(ui.available_width());

                    let (response, painter) = ui.allocate_painter(
                        egui::vec2(plot_size, plot_size),
                        egui::Sense::hover(),
                    );

                    let rect = response.rect;
                    let center = rect.center();

                    // Background
                    painter.rect_filled(rect, 5.0, Color32::from_gray(20));

                    // Grid
                    let grid_color = Color32::from_gray(40);
                    for i in -2..=2 {
                        let offset = plot_size / 4.0 * i as f32;
                        painter.line_segment(
                            [egui::pos2(center.x + offset, rect.top()), egui::pos2(center.x + offset, rect.bottom())],
                            egui::Stroke::new(1.0, grid_color),
                        );
                        painter.line_segment(
                            [egui::pos2(rect.left(), center.y + offset), egui::pos2(rect.right(), center.y + offset)],
                            egui::Stroke::new(1.0, grid_color),
                        );
                    }

                    // Axes
                    painter.line_segment(
                        [egui::pos2(rect.left(), center.y), egui::pos2(rect.right(), center.y)],
                        egui::Stroke::new(2.0, Color32::from_gray(80)),
                    );
                    painter.line_segment(
                        [egui::pos2(center.x, rect.top()), egui::pos2(center.x, rect.bottom())],
                        egui::Stroke::new(2.0, Color32::from_gray(80)),
                    );

                    // Find scale - account for noisy samples spreading out
                    let clean_max = viz.constellation.iter()
                        .map(|s| s.re.abs().max(s.im.abs()))
                        .fold(0.1f64, f64::max);

                    let noisy_max = if self.noise_enabled {
                        self.sequences.iter()
                            .flat_map(|s| s.noisy_samples.iter())
                            .map(|s| s.re.abs().max(s.im.abs()))
                            .fold(0.1f64, f64::max)
                    } else {
                        clean_max
                    };

                    let max_amp = clean_max.max(noisy_max) * 1.2; // Add margin
                    let scale = (plot_size / 2.0 - 20.0) / max_amp as f32;

                    // Draw reference constellation points
                    for (idx, point) in viz.constellation.iter().enumerate() {
                        let x = center.x + point.re as f32 * scale;
                        let y = center.y - point.im as f32 * scale;

                        // Draw point
                        painter.circle_filled(
                            egui::pos2(x, y),
                            8.0,
                            Color32::from_rgb(100, 150, 255),
                        );
                        painter.circle_stroke(
                            egui::pos2(x, y),
                            8.0,
                            egui::Stroke::new(2.0, Color32::WHITE),
                        );

                        // Draw label if available
                        if idx < viz.constellation_labels.len() {
                            let label = &viz.constellation_labels[idx];
                            painter.text(
                                egui::pos2(x + 12.0, y - 12.0),
                                egui::Align2::LEFT_BOTTOM,
                                label,
                                egui::FontId::proportional(10.0),
                                Color32::from_gray(200),
                            );
                        }
                    }

                    // Draw transmitted/received symbols for each sequence
                    for seq in &self.sequences {
                        if let Some(ref waveform) = self.current_waveform {
                            let sps = waveform.samples_per_symbol().max(1);

                            // Draw clean symbols (solid, larger)
                            if !self.noise_enabled || self.show_clean {
                                for (sym_idx, chunk) in seq.samples.chunks(sps).enumerate() {
                                    if sym_idx > 50 { break; }
                                    let mid = chunk.len() / 2;
                                    if mid < chunk.len() {
                                        let sample = chunk[mid];
                                        let x = center.x + sample.re as f32 * scale;
                                        let y = center.y - sample.im as f32 * scale;
                                        painter.circle_filled(egui::pos2(x, y), 4.0, seq.color);
                                    }
                                }
                            }

                            // Draw noisy received symbols (smaller, scattered)
                            if self.noise_enabled && self.show_noisy && !seq.noisy_samples.is_empty() {
                                let noisy_color = Color32::from_rgba_unmultiplied(
                                    255, 100, 100, 180
                                );
                                for (sym_idx, chunk) in seq.noisy_samples.chunks(sps).enumerate() {
                                    if sym_idx > 50 { break; }
                                    let mid = chunk.len() / 2;
                                    if mid < chunk.len() {
                                        let sample = chunk[mid];
                                        let x = center.x + sample.re as f32 * scale;
                                        let y = center.y - sample.im as f32 * scale;
                                        painter.circle_filled(egui::pos2(x, y), 3.0, noisy_color);
                                    }
                                }
                            }
                        }
                    }

                    // Axis labels
                    painter.text(
                        egui::pos2(rect.right() - 20.0, center.y - 15.0),
                        egui::Align2::CENTER_CENTER,
                        "I",
                        egui::FontId::proportional(14.0),
                        Color32::from_gray(150),
                    );
                    painter.text(
                        egui::pos2(center.x + 15.0, rect.top() + 20.0),
                        egui::Align2::CENTER_CENTER,
                        "Q",
                        egui::FontId::proportional(14.0),
                        Color32::from_gray(150),
                    );

                    // Constellation legend
                    ui.add_space(4.0);
                    ui.horizontal_wrapped(|ui| {
                        ui.colored_label(Color32::from_rgb(100, 150, 255), "Ideal points");
                        if self.noise_enabled {
                            ui.add_space(10.0);
                            ui.colored_label(Color32::from_rgb(255, 100, 100), "Received (noisy)");
                        }
                    });
                }
            }

            ui.add_space(20.0);
            ui.separator();

            // Educational content based on selected waveform
            ui.heading("About This Waveform");
            ui.add_space(8.0);

            match self.selected_waveform.as_str() {
                "CW" => {
                    ui.label("CW (Continuous Wave) is the simplest possible signal - a pure sinusoidal tone.");
                    ui.add_space(4.0);
                    ui.label("Key concepts:");
                    ui.indent("cw_concepts", |ui| {
                        ui.label("• Constant frequency and amplitude");
                        ui.label("• Forms a circle in the I/Q plane as phase rotates");
                        ui.label("• Building block for all other modulation schemes");
                        ui.label("• Used for Morse code (keyed on/off)");
                    });
                }
                "OOK" => {
                    ui.label("OOK (On-Off Keying) encodes data by switching the carrier on and off.");
                    ui.add_space(4.0);
                    ui.label("Key concepts:");
                    ui.indent("ook_concepts", |ui| {
                        ui.label("• '1' = carrier ON, '0' = carrier OFF");
                        ui.label("• Simplest digital modulation");
                        ui.label("• 1 bit per symbol");
                        ui.label("• Used in car remotes, garage openers, RFID");
                        ui.label("• Poor noise immunity (silence can be confused with noise)");
                    });
                    if self.noise_enabled {
                        ui.add_space(4.0);
                        ui.label("Noise performance: Poor - '0' bits (silence) can be corrupted by noise spikes");
                    }
                }
                "BFSK" | "FSK" => {
                    ui.label("FSK (Frequency Shift Keying) encodes data by shifting between frequencies.");
                    ui.add_space(4.0);
                    ui.label("Key concepts:");
                    ui.indent("fsk_concepts", |ui| {
                        ui.label("• Different frequencies represent different symbols");
                        ui.label("• Constant envelope (amplitude stays the same)");
                        ui.label("• More robust than OOK");
                        ui.label("• Used in pagers, early modems, Bluetooth (GFSK)");
                        ui.label("• Modulation index h = 2 * deviation / symbol_rate");
                    });
                    if self.noise_enabled {
                        ui.add_space(4.0);
                        ui.label("Noise performance: Good - constant envelope, non-coherent detection possible");
                    }
                }
                "BPSK" => {
                    ui.label("BPSK (Binary Phase Shift Keying) encodes data in the phase of the carrier.");
                    ui.add_space(4.0);
                    ui.label("Key concepts:");
                    ui.indent("bpsk_concepts", |ui| {
                        ui.label("• Two phases: 0° and 180°");
                        ui.label("• 1 bit per symbol");
                        ui.label("• Constant envelope");
                        ui.label("• Very robust - used in deep space communications");
                        ui.label("• Constellation: two points on the real axis");
                    });
                    if self.noise_enabled {
                        ui.add_space(4.0);
                        ui.label("Noise performance: Excellent - maximum distance between constellation points");
                    }
                }
                "QPSK" => {
                    ui.label("QPSK (Quadrature PSK) uses 4 phases to encode 2 bits per symbol.");
                    ui.add_space(4.0);
                    ui.label("Key concepts:");
                    ui.indent("qpsk_concepts", |ui| {
                        ui.label("• Four phases: 45°, 135°, 225°, 315°");
                        ui.label("• 2 bits per symbol - double the efficiency of BPSK");
                        ui.label("• Gray coding: adjacent points differ by only 1 bit");
                        ui.label("• Used in WiFi, satellites, cellular");
                        ui.label("• Constellation: four points in a square pattern");
                    });
                    if self.noise_enabled {
                        ui.add_space(4.0);
                        ui.label("Noise performance: Very good - same power efficiency as BPSK with 2x data rate");
                    }
                }
                "16QAM" | "QAM16" => {
                    ui.label("16-QAM combines amplitude AND phase to encode 4 bits per symbol.");
                    ui.add_space(4.0);
                    ui.label("Key concepts:");
                    ui.indent("qam_concepts", |ui| {
                        ui.label("• 16 constellation points in a 4x4 grid");
                        ui.label("• 4 bits per symbol - high spectral efficiency");
                        ui.label("• Requires higher SNR than PSK (~17 dB)");
                        ui.label("• Used in WiFi, LTE, cable modems");
                        ui.label("• Trade-off: more bits = need cleaner channel");
                    });
                    if self.noise_enabled {
                        ui.add_space(4.0);
                        ui.label("Noise performance: Moderate - closer constellation points mean higher error rate at same SNR");
                    }
                }
                _ => {
                    ui.label("Select a waveform to learn more about it.");
                }
            }

            // Noise comparison tip
            if self.noise_enabled {
                ui.add_space(12.0);
                ui.separator();
                ui.heading("Comparing Waveform Performance");
                ui.add_space(4.0);
                ui.label("Try these experiments:");
                ui.indent("noise_tips", |ui| {
                    ui.label("• Compare BPSK vs 16QAM at the same SNR - notice QAM has more errors");
                    ui.label("• Lower SNR to see when each waveform starts failing");
                    ui.label("• OOK performs poorly because '0' (silence) gets corrupted by noise");
                    ui.label("• PSK maintains good performance due to constant envelope");
                    ui.label("• Watch the constellation scatter - points spread out with more noise");
                });
            }
        });
    }
}
