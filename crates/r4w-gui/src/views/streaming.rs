//! Real-time streaming view with oscilloscope and waterfall display

use crate::platform::{platform, PlatformServices};
use crate::streaming::{ChannelModel, PlaybackState, StreamManager, StreamSource, UdpSampleFormat, UdpStatus};
use crate::streaming::waterfall::{viridis_color, jet_color};
use byteorder::{LittleEndian, ReadBytesExt};
use egui::{Color32, Pos2, Rect, Ui, Vec2};
use egui_plot::{Line, Plot, PlotPoints, Points};
use r4w_core::types::IQSample;
use std::path::PathBuf;

/// Colormap selection for waterfall
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Colormap {
    Viridis,
    Jet,
}

impl Default for Colormap {
    fn default() -> Self {
        Self::Viridis
    }
}

/// Spectrum display mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SpectrumMode {
    /// Show current FFT frame only
    #[default]
    Normal,
    /// Average multiple frames
    Average,
    /// Show maximum value at each bin
    MaxHold,
}

/// Streaming view state
pub struct StreamingView {
    /// Show I component in oscilloscope
    show_i: bool,
    /// Show Q component in oscilloscope
    show_q: bool,
    /// Show TX signal in simulation mode
    show_tx: bool,
    /// Show constellation diagram
    show_constellation: bool,
    /// Number of points to show in constellation
    constellation_points: usize,
    /// Show BER plot
    show_ber_plot: bool,
    /// Show eye diagram
    show_eye_diagram: bool,
    /// Samples per symbol for eye diagram (user-adjustable)
    eye_samples_per_symbol: usize,
    /// Number of symbol periods to overlay in eye diagram
    eye_num_traces: usize,
    /// Selected colormap for waterfall
    colormap: Colormap,
    /// Generator waveform selection
    selected_generator: String,
    /// Available generator waveforms
    generator_options: Vec<String>,
    /// Simulation SNR in dB
    sim_snr_db: f32,
    /// Spectrum display mode (Normal, Average, MaxHold)
    spectrum_mode: SpectrumMode,
    /// Accumulated spectrum for averaging/max hold
    spectrum_accum: Vec<f32>,
    /// Number of frames accumulated (for averaging)
    spectrum_frame_count: usize,
    /// Averaging window size
    spectrum_avg_count: usize,
    /// Marker 1 position (frequency_khz, power_db)
    spectrum_marker1: Option<(f64, f32)>,
    /// Marker 2 position (frequency_khz, power_db)
    spectrum_marker2: Option<(f64, f32)>,
    /// UDP port number for network streaming (native only)
    #[cfg(not(target_arch = "wasm32"))]
    udp_port: u16,
    /// UDP sample format (native only)
    #[cfg(not(target_arch = "wasm32"))]
    udp_format: UdpSampleFormat,
}

impl Default for StreamingView {
    fn default() -> Self {
        Self::new()
    }
}

impl StreamingView {
    pub fn new() -> Self {
        // Get all available waveforms from the factory
        let generator_options: Vec<String> = StreamManager::available_waveforms()
            .into_iter()
            .map(|s| s.to_string())
            .collect();

        Self {
            show_i: true,
            show_q: true,
            show_tx: true,
            show_constellation: true,
            constellation_points: 512,
            show_ber_plot: true,
            show_eye_diagram: true,
            eye_samples_per_symbol: 16,  // Default: 16 samples per symbol
            eye_num_traces: 50,          // Default: overlay 50 symbol periods
            colormap: Colormap::Viridis,
            selected_generator: "BPSK".to_string(),
            generator_options,
            sim_snr_db: 10.0,
            spectrum_mode: SpectrumMode::Normal,
            spectrum_accum: Vec::new(),
            spectrum_frame_count: 0,
            spectrum_avg_count: 10, // Average over 10 frames by default
            spectrum_marker1: None,
            spectrum_marker2: None,
            #[cfg(not(target_arch = "wasm32"))]
            udp_port: 5000, // Default UDP port
            #[cfg(not(target_arch = "wasm32"))]
            udp_format: UdpSampleFormat::Float32,
        }
    }

    /// Sync the generator selection with the sidebar waveform
    /// Returns true if the generator was changed
    pub fn sync_with_sidebar(&mut self, sidebar_waveform: &str) -> bool {
        // Map sidebar waveform names to streaming generator names
        let generator_name = match sidebar_waveform {
            // Direct matches (most waveforms)
            "CW" | "OOK" | "PPM" | "ADS-B" |
            "BFSK" | "4-FSK" | "BPSK" | "QPSK" | "8-PSK" |
            "16-QAM" | "64-QAM" | "256-QAM" |
            "AM" | "FM" | "OFDM" |
            "DSSS" | "DSSS-QPSK" | "FHSS" |
            "Zigbee" | "UWB" | "FMCW" => sidebar_waveform.to_string(),

            // LoRa is not available in streaming - use DSSS as closest alternative
            "LoRa" => {
                // LoRa/CSS isn't in WaveformFactory, so we don't change
                return false;
            }

            // Unknown waveform - don't change
            _ => return false,
        };

        // Check if this generator is available
        if self.generator_options.contains(&generator_name) {
            if self.selected_generator != generator_name {
                self.selected_generator = generator_name;
                return true;
            }
        }

        false
    }

    /// Get the currently selected generator name
    pub fn selected_generator(&self) -> &str {
        &self.selected_generator
    }

    /// Render the streaming view
    pub fn render(&mut self, ui: &mut Ui, manager: &mut StreamManager) {
        ui.heading("Real-Time Streaming");
        ui.add_space(4.0);

        // Source selection and controls
        self.render_source_controls(ui, manager);
        ui.add_space(8.0);

        // Playback controls
        self.render_playback_controls(ui, manager);
        ui.add_space(8.0);

        // Position scrubber (file mode only)
        if matches!(manager.source, StreamSource::File { .. }) {
            self.render_position_scrubber(ui, manager);
            ui.add_space(8.0);
        }

        // Recording controls (save/clear) - shown when samples are recorded
        self.render_recording_controls(ui, manager);
        ui.add_space(4.0);

        ui.separator();
        ui.add_space(8.0);

        // Display options
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.show_i, "I (In-phase)");
            ui.checkbox(&mut self.show_q, "Q (Quadrature)");
            if manager.is_simulation() {
                ui.separator();
                ui.checkbox(&mut self.show_tx, "TX Signal");
                ui.checkbox(&mut self.show_ber_plot, "BER Plot");
            }
            ui.separator();
            ui.checkbox(&mut self.show_constellation, "Constellation");
            ui.checkbox(&mut self.show_eye_diagram, "Eye Diagram");
            ui.separator();
            ui.label("Colormap:");
            egui::ComboBox::from_id_salt("colormap")
                .selected_text(format!("{:?}", self.colormap))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.colormap, Colormap::Viridis, "Viridis");
                    ui.selectable_value(&mut self.colormap, Colormap::Jet, "Jet");
                });
        });

        ui.add_space(8.0);

        // Oscilloscope, Constellation, and Eye Diagram side by side
        // Calculate widths before layout
        let total_width = ui.available_width();
        let side_panel_width = 260.0;
        let num_side_panels = (if self.show_constellation { 1 } else { 0 })
            + (if self.show_eye_diagram { 1 } else { 0 });
        let osc_width = if num_side_panels > 0 {
            (total_width - (num_side_panels as f32 * side_panel_width) - 20.0).max(300.0)
        } else {
            total_width
        };

        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.heading("Time Domain (Oscilloscope)");
                ui.set_max_width(osc_width);
                self.render_oscilloscope(ui, manager);
            });

            if self.show_constellation {
                ui.separator();
                ui.vertical(|ui| {
                    ui.heading("Constellation (I vs Q)");
                    self.render_constellation(ui, manager);
                });
            }

            if self.show_eye_diagram {
                ui.separator();
                ui.vertical(|ui| {
                    ui.heading("Eye Diagram");
                    self.render_eye_diagram(ui, manager);
                });
            }
        });

        ui.add_space(12.0);

        // Waterfall (frequency vs time)
        ui.heading("Waterfall Spectrogram");
        self.render_waterfall(ui, manager);

        // Spectrum plot (current FFT frame) below waterfall
        self.render_spectrum_plot(ui, manager);

        ui.add_space(12.0);

        // BER plot (simulation mode only)
        if manager.is_simulation() && self.show_ber_plot {
            self.render_ber_plot(ui, manager);
            ui.add_space(12.0);
        }

        // Statistics
        self.render_statistics(ui, manager);
    }

    /// Render source selection controls
    fn render_source_controls(&mut self, ui: &mut Ui, manager: &mut StreamManager) {
        ui.horizontal(|ui| {
            ui.label("Source:");

            // Generator button
            if ui.button("Generator").clicked() {
                manager.start_generator(self.selected_generator.clone());
            }

            // Generator type dropdown
            egui::ComboBox::from_id_salt("generator_type")
                .selected_text(&self.selected_generator)
                .show_ui(ui, |ui| {
                    for opt in &self.generator_options {
                        ui.selectable_value(&mut self.selected_generator, opt.clone(), opt);
                    }
                });

            ui.separator();

            // Load file button - only available on native
            let file_ops = platform().file_ops_available();
            if ui.add_enabled(file_ops, egui::Button::new("Load File..."))
                .on_hover_text(if file_ops {
                    "Load IQ sample file"
                } else {
                    "File loading not available in web version"
                })
                .clicked()
            {
                match platform().pick_iq_file() {
                    Ok(loaded) => {
                        match self.load_iq_data(&loaded.path, &loaded.data, manager) {
                            Ok(count) => {
                                tracing::info!("Loaded {} samples from {:?}", count, loaded.path);
                            }
                            Err(e) => {
                                tracing::error!("Failed to parse file: {}", e);
                            }
                        }
                    }
                    Err(crate::platform::FileError::Cancelled) => {
                        // User cancelled, no action needed
                    }
                    Err(e) => {
                        tracing::error!("Failed to load file: {}", e);
                    }
                }
            }

            // Demo signal button for testing
            if ui.button("Demo Signal").clicked() {
                self.load_test_signal(manager);
            }

            ui.separator();

            // Simulation button
            let sim_btn = egui::Button::new("TX→RX Sim")
                .fill(if manager.is_simulation() { Color32::from_rgb(60, 120, 60) } else { Color32::TRANSPARENT });
            if ui.add(sim_btn).on_hover_text("Start TX→Channel→RX simulation").clicked() {
                manager.start_simulation(self.selected_generator.clone(), self.sim_snr_db as f64);
            }

            // UDP controls (native only - not available in WASM)
            #[cfg(not(target_arch = "wasm32"))]
            {
                ui.separator();

                // Start/Stop UDP button
                let udp_active = manager.is_udp();
                let udp_btn = egui::Button::new(if udp_active { "Stop UDP" } else { "UDP" })
                    .fill(if udp_active { Color32::from_rgb(180, 60, 60) } else { Color32::TRANSPARENT });
                if ui.add(udp_btn).on_hover_text(if udp_active {
                    "Stop UDP listener"
                } else {
                    "Start UDP listener"
                }).clicked() {
                    if udp_active {
                        manager.stop_udp();
                    } else {
                        match manager.start_udp(self.udp_port, self.udp_format) {
                            Ok(()) => tracing::info!("UDP listener started on port {}", self.udp_port),
                            Err(e) => tracing::error!("Failed to start UDP: {}", e),
                        }
                    }
                }

                // Port number input (only when not active)
                if !udp_active {
                    ui.add(egui::DragValue::new(&mut self.udp_port)
                        .range(1024..=65535)
                        .prefix("Port: "));

                    // Format selector
                    egui::ComboBox::from_id_salt("udp_format")
                        .selected_text(self.udp_format.name())
                        .width(50.0)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.udp_format, UdpSampleFormat::Float32, "f32");
                            ui.selectable_value(&mut self.udp_format, UdpSampleFormat::Int16, "i16");
                        });
                }
            }

            ui.separator();

            // Current source display
            ui.label(format!("Current: {}", manager.source_description()));
        });

        // Second row: Sample rate and simulation config
        ui.horizontal(|ui| {
            ui.label("Sample Rate:");
            let mut sample_rate_khz = (manager.config.sample_rate / 1000.0) as f32;
            let slider = egui::Slider::new(&mut sample_rate_khz, 8.0..=500.0)
                .suffix(" kHz")
                .logarithmic(true);
            if ui.add(slider).changed() {
                manager.config.sample_rate = sample_rate_khz as f64 * 1000.0;
                // Re-initialize source with new sample rate if active
                if matches!(manager.source, StreamSource::Generator { .. }) {
                    manager.start_generator(self.selected_generator.clone());
                } else if matches!(manager.source, StreamSource::Simulation { .. }) {
                    manager.start_simulation(self.selected_generator.clone(), self.sim_snr_db as f64);
                }
            }

            ui.separator();

            // SNR control (always visible for easy configuration)
            ui.label("SNR:");
            let snr_slider = egui::Slider::new(&mut self.sim_snr_db, -10.0..=30.0)
                .suffix(" dB")
                .fixed_decimals(1);
            if ui.add(snr_slider).changed() && manager.is_simulation() {
                // Update running simulation's SNR
                manager.sim_snr_db = self.sim_snr_db as f64;
            }

            // Channel model selector
            ui.separator();
            ui.label("Channel:");
            egui::ComboBox::from_id_salt("channel_model")
                .selected_text(format!("{:?}", manager.sim_channel_model))
                .width(80.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut manager.sim_channel_model, ChannelModel::Ideal, "Ideal");
                    ui.selectable_value(&mut manager.sim_channel_model, ChannelModel::AWGN, "AWGN");
                    ui.selectable_value(&mut manager.sim_channel_model, ChannelModel::Rayleigh, "Rayleigh");
                    ui.selectable_value(&mut manager.sim_channel_model, ChannelModel::Rician, "Rician");
                });

            // Rician K-factor slider (only when Rician is selected)
            if manager.sim_channel_model == ChannelModel::Rician {
                ui.separator();
                ui.label("K:");
                let mut k_db = manager.sim_rician_k_db as f32;
                let k_slider = egui::Slider::new(&mut k_db, -10.0..=20.0)
                    .suffix(" dB")
                    .fixed_decimals(1);
                if ui.add(k_slider).on_hover_text("Rician K-factor (LOS/scatter ratio)").changed() {
                    manager.sim_rician_k_db = k_db as f64;
                }
            }

            // Carrier frequency offset slider
            ui.separator();
            ui.label("CFO:");
            let mut cfo_hz = manager.sim_cfo_hz as f32;
            let cfo_slider = egui::Slider::new(&mut cfo_hz, -1000.0..=1000.0)
                .suffix(" Hz")
                .fixed_decimals(0);
            if ui.add(cfo_slider).on_hover_text("Carrier Frequency Offset").changed() {
                manager.sim_cfo_hz = cfo_hz as f64;
            }

            // Phase noise slider
            ui.separator();
            ui.label("Phase Noise:");
            let mut phase_noise = manager.sim_phase_noise_deg as f32;
            let pn_slider = egui::Slider::new(&mut phase_noise, 0.0..=10.0)
                .suffix("°/samp")
                .fixed_decimals(2);
            if ui.add(pn_slider).on_hover_text("Phase noise std dev (Wiener process)").changed() {
                manager.sim_phase_noise_deg = phase_noise as f64;
            }
        });

        // Third row: IQ imbalance controls
        ui.horizontal(|ui| {
            ui.label("IQ Imbalance:");

            // Amplitude imbalance (Q gain relative to I)
            ui.label("Gain:");
            let mut iq_gain = manager.sim_iq_gain_db as f32;
            let gain_slider = egui::Slider::new(&mut iq_gain, -3.0..=3.0)
                .suffix(" dB")
                .fixed_decimals(2);
            if ui.add(gain_slider).on_hover_text("Q branch gain relative to I (amplitude imbalance)").changed() {
                manager.sim_iq_gain_db = iq_gain as f64;
            }

            ui.separator();

            // Phase imbalance (deviation from 90°)
            ui.label("Phase:");
            let mut iq_phase = manager.sim_iq_phase_deg as f32;
            let phase_slider = egui::Slider::new(&mut iq_phase, -10.0..=10.0)
                .suffix("°")
                .fixed_decimals(1);
            if ui.add(phase_slider).on_hover_text("Phase deviation from 90° (phase imbalance)").changed() {
                manager.sim_iq_phase_deg = iq_phase as f64;
            }

            ui.separator();

            // DC offset controls
            ui.label("DC:");
            ui.label("I:");
            let mut dc_i = manager.sim_dc_offset_i as f32;
            let dc_i_slider = egui::Slider::new(&mut dc_i, -0.5..=0.5)
                .fixed_decimals(3);
            if ui.add(dc_i_slider).on_hover_text("DC offset on I channel").changed() {
                manager.sim_dc_offset_i = dc_i as f64;
            }

            ui.label("Q:");
            let mut dc_q = manager.sim_dc_offset_q as f32;
            let dc_q_slider = egui::Slider::new(&mut dc_q, -0.5..=0.5)
                .fixed_decimals(3);
            if ui.add(dc_q_slider).on_hover_text("DC offset on Q channel").changed() {
                manager.sim_dc_offset_q = dc_q as f64;
            }
        });
    }

    /// Load IQ samples from raw bytes (f32 little-endian interleaved I/Q pairs)
    fn load_iq_data(&self, path: &PathBuf, data: &[u8], manager: &mut StreamManager) -> Result<usize, String> {
        let file_size = data.len();

        // Each sample is 8 bytes (2 x f32)
        let expected_samples = file_size / 8;
        if file_size % 8 != 0 {
            tracing::warn!("Data size not aligned to 8 bytes, trailing bytes will be ignored");
        }

        let mut reader = std::io::Cursor::new(data);
        let mut samples = Vec::with_capacity(expected_samples);

        loop {
            let i = match reader.read_f32::<LittleEndian>() {
                Ok(v) => v as f64,
                Err(ref e) if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
                Err(e) => return Err(format!("Failed to read I component: {}", e)),
            };

            let q = match reader.read_f32::<LittleEndian>() {
                Ok(v) => v as f64,
                Err(ref e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                    tracing::warn!("Incomplete sample at end of data");
                    break;
                }
                Err(e) => return Err(format!("Failed to read Q component: {}", e)),
            };

            samples.push(IQSample::new(i, q));
        }

        let count = samples.len();
        if count == 0 {
            return Err("No samples found in data".to_string());
        }

        manager.load_file(path.clone(), samples);
        Ok(count)
    }

    /// Load a test signal for demonstration
    fn load_test_signal(&self, manager: &mut StreamManager) {

        // Generate a test chirp signal
        let sample_rate = manager.config.sample_rate;
        let duration = 2.0; // 2 seconds
        let num_samples = (sample_rate * duration) as usize;

        let mut samples = Vec::with_capacity(num_samples);
        let f0 = 0.0;
        let f1 = 10000.0;
        let chirp_period = 0.1; // 100ms chirp

        for i in 0..num_samples {
            let t = i as f64 / sample_rate;
            let t_mod = t % chirp_period;
            let _freq = f0 + (f1 - f0) * (t_mod / chirp_period);
            let phase = 2.0 * std::f64::consts::PI * (f0 * t_mod + 0.5 * (f1 - f0) / chirp_period * t_mod * t_mod);
            samples.push(IQSample::new(phase.cos(), phase.sin()));
        }

        manager.load_file(PathBuf::from("test_chirp.iq"), samples);
    }

    /// Render playback controls
    fn render_playback_controls(&mut self, ui: &mut Ui, manager: &mut StreamManager) {
        ui.horizontal(|ui| {
            // Rewind button
            if ui.button("|<").on_hover_text("Rewind").clicked() {
                manager.stop();
            }

            // Play/Pause button
            let play_text = match manager.state {
                PlaybackState::Playing => "||",
                _ => ">",
            };
            let play_hover = match manager.state {
                PlaybackState::Playing => "Pause",
                _ => "Play",
            };
            if ui.button(play_text).on_hover_text(play_hover).clicked() {
                manager.toggle_play();
            }

            // Stop button
            if ui.button("Stop").clicked() {
                manager.stop();
            }

            ui.separator();

            // Speed control (0.001x = 1000x slowdown for educational viewing)
            ui.label("Speed:");
            let mut speed = manager.config.playback_speed;
            let speed_slider = egui::Slider::new(&mut speed, 0.001..=10.0)
                .logarithmic(true)
                .suffix("x")
                .fixed_decimals(3);
            if ui.add(speed_slider).changed() {
                manager.config.playback_speed = speed;
            }

            ui.separator();

            // State indicator
            let state_text = match manager.state {
                PlaybackState::Stopped => "Stopped",
                PlaybackState::Playing => "Playing",
                PlaybackState::Paused => "Paused",
            };
            let state_color = match manager.state {
                PlaybackState::Stopped => Color32::GRAY,
                PlaybackState::Playing => Color32::GREEN,
                PlaybackState::Paused => Color32::YELLOW,
            };
            ui.colored_label(state_color, state_text);

            ui.separator();

            // Recording controls
            if manager.is_recording() {
                // Stop recording button (red)
                let stop_btn = egui::Button::new("⏹ Stop Rec")
                    .fill(Color32::from_rgb(180, 60, 60));
                if ui.add(stop_btn).on_hover_text("Stop recording").clicked() {
                    manager.stop_recording();
                }
                // Recording indicator
                ui.colored_label(Color32::RED, format!("● REC {:.1}s", manager.recording_duration()));
            } else {
                // Start recording button
                let rec_btn = egui::Button::new("⏺ Record");
                if ui.add(rec_btn).on_hover_text("Start recording samples").clicked() {
                    manager.start_recording();
                }
                // Show recorded samples count if any
                if manager.recorded_samples() > 0 {
                    ui.label(format!("{} samples", manager.recorded_samples()));
                }
            }
        });
    }

    /// Render recording controls (save/clear)
    fn render_recording_controls(&mut self, ui: &mut Ui, manager: &mut StreamManager) {
        if manager.recorded_samples() == 0 && !manager.is_recording() {
            return;
        }

        ui.horizontal(|ui| {
            ui.label("Recording:");

            // Recording progress bar
            let capacity_used = 1.0 - manager.recording_capacity_remaining();
            let desired_size = Vec2::new(150.0, 16.0);
            let (rect, _) = ui.allocate_exact_size(desired_size, egui::Sense::hover());

            // Draw background
            ui.painter().rect_filled(rect, 2.0, Color32::from_gray(40));

            // Draw progress
            let progress_width = rect.width() * capacity_used;
            let progress_rect = Rect::from_min_size(rect.min, Vec2::new(progress_width, rect.height()));
            let bar_color = if manager.is_recording() {
                Color32::from_rgb(200, 60, 60)
            } else {
                Color32::from_rgb(100, 100, 200)
            };
            ui.painter().rect_filled(progress_rect, 2.0, bar_color);

            // Stats
            ui.label(format!(
                "{} samples ({:.1}s)",
                manager.recorded_samples(),
                manager.recording_duration()
            ));

            ui.separator();

            // Save button - only available on native
            let file_ops = platform().file_ops_available();
            if ui.add_enabled(file_ops, egui::Button::new("Save..."))
                .on_hover_text(if file_ops {
                    "Save recording to IQ file"
                } else {
                    "File saving not available in web version"
                })
                .clicked()
            {
                // Get the recording data from manager
                let data = manager.get_recording_bytes();
                match platform().save_iq_file("recording.iq", &data) {
                    Ok(()) => {
                        tracing::info!("Saved {} bytes", data.len());
                    }
                    Err(crate::platform::FileError::Cancelled) => {
                        // User cancelled, no action needed
                    }
                    Err(e) => {
                        tracing::error!("Failed to save recording: {}", e);
                    }
                }
            }

            // Clear button
            if ui.button("Clear").on_hover_text("Discard recorded samples").clicked() {
                manager.clear_recording();
            }
        });
    }

    /// Render position scrubber for file playback
    fn render_position_scrubber(&mut self, ui: &mut Ui, manager: &mut StreamManager) {
        let progress = manager.progress();
        let elapsed = manager.elapsed_seconds();
        let total = manager.total_seconds();

        ui.horizontal(|ui| {
            ui.label("Position:");

            // Progress bar that doubles as scrubber
            let desired_size = Vec2::new(ui.available_width() - 120.0, 20.0);
            let (rect, response) = ui.allocate_exact_size(desired_size, egui::Sense::click_and_drag());

            // Draw background
            ui.painter().rect_filled(rect, 2.0, Color32::from_gray(40));

            // Draw progress
            let progress_width = rect.width() * progress;
            let progress_rect = Rect::from_min_size(rect.min, Vec2::new(progress_width, rect.height()));
            ui.painter().rect_filled(progress_rect, 2.0, Color32::from_rgb(70, 130, 180));

            // Handle click/drag for seeking
            if response.clicked() || response.dragged() {
                if let Some(pos) = response.interact_pointer_pos() {
                    let new_progress = ((pos.x - rect.left()) / rect.width()).clamp(0.0, 1.0);
                    let new_position = (new_progress as f64 * manager.stats.total_samples as f64) as usize;
                    manager.seek(new_position);
                }
            }

            // Time display
            ui.label(format!("{:.1}s / {:.1}s", elapsed, total));
        });
    }

    /// Render oscilloscope (time domain) display
    fn render_oscilloscope(&mut self, ui: &mut Ui, manager: &StreamManager) {
        let rx_samples = manager.time_buffer.as_slice_ordered();
        let tx_samples = manager.tx_buffer.as_slice_ordered();
        let is_simulation = manager.is_simulation();

        if rx_samples.is_empty() {
            ui.colored_label(Color32::YELLOW, "No samples. Start playback to see waveform.");
            // Still show empty plot
            let plot = Plot::new("oscilloscope")
                .height(200.0)
                .allow_zoom(true)
                .allow_drag(true)
                .x_axis_label("Sample")
                .y_axis_label("Amplitude");
            plot.show(ui, |_plot_ui| {});
            return;
        }

        // Show TX vs RX label in simulation mode
        if is_simulation {
            ui.horizontal(|ui| {
                ui.colored_label(Color32::from_rgb(100, 200, 255), "● RX (received)");
                if self.show_tx {
                    ui.colored_label(Color32::from_rgb(100, 255, 100), "● TX (transmitted)");
                }
            });
        }

        let plot = Plot::new("oscilloscope")
            .height(200.0)
            .allow_zoom(true)
            .allow_drag(true)
            .x_axis_label("Sample")
            .y_axis_label("Amplitude");

        plot.show(ui, |plot_ui| {
            // Show TX signal first (behind RX) in simulation mode
            if is_simulation && self.show_tx && !tx_samples.is_empty() {
                if self.show_i {
                    let tx_i_points: PlotPoints = tx_samples
                        .iter()
                        .enumerate()
                        .map(|(i, s)| [i as f64, s.re])
                        .collect();
                    plot_ui.line(
                        Line::new(tx_i_points)
                            .name("TX I")
                            .color(Color32::from_rgb(100, 255, 100).gamma_multiply(0.6)),
                    );
                }
                if self.show_q {
                    let tx_q_points: PlotPoints = tx_samples
                        .iter()
                        .enumerate()
                        .map(|(i, s)| [i as f64, s.im])
                        .collect();
                    plot_ui.line(
                        Line::new(tx_q_points)
                            .name("TX Q")
                            .color(Color32::from_rgb(200, 255, 100).gamma_multiply(0.6)),
                    );
                }
            }

            // RX signal (or regular signal in non-simulation mode)
            if self.show_i {
                let i_points: PlotPoints = rx_samples
                    .iter()
                    .enumerate()
                    .map(|(i, s)| [i as f64, s.re])
                    .collect();
                let name = if is_simulation { "RX I" } else { "I" };
                plot_ui.line(
                    Line::new(i_points)
                        .name(name)
                        .color(Color32::from_rgb(100, 200, 255)),
                );
            }

            if self.show_q {
                let q_points: PlotPoints = rx_samples
                    .iter()
                    .enumerate()
                    .map(|(i, s)| [i as f64, s.im])
                    .collect();
                let name = if is_simulation { "RX Q" } else { "Q" };
                plot_ui.line(
                    Line::new(q_points)
                        .name(name)
                        .color(Color32::from_rgb(255, 150, 100)),
                );
            }
        });
    }

    /// Render constellation diagram (I vs Q scatter plot)
    fn render_constellation(&mut self, ui: &mut Ui, manager: &StreamManager) {
        let rx_samples = manager.time_buffer.as_slice_ordered();
        let tx_samples = manager.tx_buffer.as_slice_ordered();
        let is_simulation = manager.is_simulation();

        // Take the last N points for constellation
        let num_points = self.constellation_points.min(rx_samples.len());

        let plot = Plot::new("constellation")
            .height(200.0)
            .width(200.0)
            .data_aspect(1.0)  // Square aspect ratio
            .allow_zoom(true)
            .allow_drag(true)
            .x_axis_label("I (In-phase)")
            .y_axis_label("Q (Quadrature)")
            .include_x(-1.5)
            .include_x(1.5)
            .include_y(-1.5)
            .include_y(1.5);

        plot.show(ui, |plot_ui| {
            if rx_samples.is_empty() {
                return;
            }

            // Show TX constellation in simulation mode (ideal points)
            if is_simulation && self.show_tx && !tx_samples.is_empty() {
                let tx_num = num_points.min(tx_samples.len());
                let tx_constellation: PlotPoints = tx_samples
                    .iter()
                    .rev()
                    .take(tx_num)
                    .map(|s| [s.re, s.im])
                    .collect();
                plot_ui.points(
                    Points::new(tx_constellation)
                        .name("TX")
                        .color(Color32::from_rgb(100, 255, 100).gamma_multiply(0.5))
                        .radius(2.0),
                );
            }

            // RX constellation (with impairments)
            let rx_constellation: PlotPoints = rx_samples
                .iter()
                .rev()
                .take(num_points)
                .map(|s| [s.re, s.im])
                .collect();
            plot_ui.points(
                Points::new(rx_constellation)
                    .name("RX")
                    .color(Color32::from_rgb(100, 200, 255))
                    .radius(2.0),
            );
        });

        // Constellation point count control
        ui.horizontal(|ui| {
            ui.label("Points:");
            let mut points = self.constellation_points as u32;
            if ui.add(egui::Slider::new(&mut points, 64..=2048).logarithmic(true)).changed() {
                self.constellation_points = points as usize;
            }
        });
    }

    /// Render eye diagram
    ///
    /// An eye diagram overlays multiple symbol periods to visualize:
    /// - Timing jitter (horizontal eye opening)
    /// - Noise margin (vertical eye opening)
    /// - Inter-symbol interference (ISI)
    fn render_eye_diagram(&mut self, ui: &mut Ui, manager: &StreamManager) {
        let samples = manager.time_buffer.as_slice_ordered();
        let sps = self.eye_samples_per_symbol;
        let num_traces = self.eye_num_traces;

        // Need at least 2 symbol periods to show an eye
        let min_samples = sps * 2;
        if samples.len() < min_samples {
            ui.colored_label(Color32::YELLOW, "Insufficient samples for eye diagram");
            // Show empty plot
            let plot = Plot::new("eye_diagram")
                .height(200.0)
                .width(250.0)
                .allow_zoom(true)
                .allow_drag(true)
                .x_axis_label("Time (samples)")
                .y_axis_label("Amplitude");
            plot.show(ui, |_plot_ui| {});

            // Still show controls
            self.render_eye_controls(ui);
            return;
        }

        // Calculate how many complete symbol periods we can show
        // We show 2 symbol periods per trace (classic eye diagram)
        let trace_length = sps * 2;
        let available_traces = (samples.len() - trace_length) / sps;
        let traces_to_show = num_traces.min(available_traces).max(1);

        let plot = Plot::new("eye_diagram")
            .height(200.0)
            .width(250.0)
            .allow_zoom(true)
            .allow_drag(true)
            .x_axis_label("Time (samples)")
            .y_axis_label("Amplitude")
            .include_y(-1.5)
            .include_y(1.5);

        plot.show(ui, |plot_ui| {
            // Overlay multiple traces
            for trace_idx in 0..traces_to_show {
                let start = trace_idx * sps;
                let end = start + trace_length;

                if end > samples.len() {
                    break;
                }

                // Plot I component (blue with transparency for overlay effect)
                if self.show_i {
                    let i_points: PlotPoints = samples[start..end]
                        .iter()
                        .enumerate()
                        .map(|(i, s)| [i as f64, s.re])
                        .collect();

                    // Use semi-transparent color for overlay effect
                    let alpha = (128.0 * (1.0 - trace_idx as f32 / traces_to_show as f32) + 50.0) as u8;
                    plot_ui.line(
                        Line::new(i_points)
                            .color(Color32::from_rgba_unmultiplied(100, 200, 255, alpha))
                            .width(1.0),
                    );
                }

                // Plot Q component (orange with transparency)
                if self.show_q {
                    let q_points: PlotPoints = samples[start..end]
                        .iter()
                        .enumerate()
                        .map(|(i, s)| [i as f64, s.im])
                        .collect();

                    let alpha = (128.0 * (1.0 - trace_idx as f32 / traces_to_show as f32) + 50.0) as u8;
                    plot_ui.line(
                        Line::new(q_points)
                            .color(Color32::from_rgba_unmultiplied(255, 150, 100, alpha))
                            .width(1.0),
                    );
                }
            }

            // Draw vertical lines at symbol boundaries
            let boundary_color = Color32::from_rgba_unmultiplied(200, 200, 200, 80);
            for i in 0..=2 {
                let x = (i * sps) as f64;
                let boundary: PlotPoints = vec![[x, -2.0], [x, 2.0]].into();
                plot_ui.line(
                    Line::new(boundary)
                        .color(boundary_color)
                        .style(egui_plot::LineStyle::Dashed { length: 3.0 }),
                );
            }

            // Draw horizontal zero line
            let zero_line: PlotPoints = vec![[0.0, 0.0], [trace_length as f64, 0.0]].into();
            plot_ui.line(
                Line::new(zero_line)
                    .color(Color32::from_rgba_unmultiplied(150, 150, 150, 60))
                    .style(egui_plot::LineStyle::Dashed { length: 5.0 }),
            );
        });

        // Controls
        self.render_eye_controls(ui);
    }

    /// Render eye diagram controls
    fn render_eye_controls(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.label("Samp/Sym:");
            let mut sps = self.eye_samples_per_symbol as u32;
            if ui.add(egui::Slider::new(&mut sps, 4..=128).logarithmic(true)).changed() {
                self.eye_samples_per_symbol = sps as usize;
            }
        });
        ui.horizontal(|ui| {
            ui.label("Traces:");
            let mut traces = self.eye_num_traces as u32;
            if ui.add(egui::Slider::new(&mut traces, 10..=200)).changed() {
                self.eye_num_traces = traces as usize;
            }
        });
    }

    /// Render BER plot (simulation mode)
    fn render_ber_plot(&self, ui: &mut Ui, manager: &StreamManager) {
        ui.heading("Bit Error Rate (BER)");

        // BER summary row
        ui.horizontal(|ui| {
            // Cumulative BER with color coding
            let ber_color = if manager.sim_ber < 0.001 {
                Color32::GREEN
            } else if manager.sim_ber < 0.01 {
                Color32::YELLOW
            } else if manager.sim_ber < 0.1 {
                Color32::from_rgb(255, 165, 0) // Orange
            } else {
                Color32::RED
            };

            ui.label("Cumulative:");
            ui.colored_label(ber_color, format!("{:.2e}", manager.sim_ber));
            ui.separator();

            // Window BER
            let window_color = if manager.sim_ber_window < 0.001 {
                Color32::GREEN
            } else if manager.sim_ber_window < 0.01 {
                Color32::YELLOW
            } else if manager.sim_ber_window < 0.1 {
                Color32::from_rgb(255, 165, 0)
            } else {
                Color32::RED
            };
            ui.label("Window:");
            ui.colored_label(window_color, format!("{:.2e}", manager.sim_ber_window));
            ui.separator();

            // Error count
            ui.label(format!(
                "Errors: {} / {} bits",
                manager.sim_bit_errors,
                manager.sim_tx_bits.len().min(manager.sim_demod_bits.len())
            ));
        });

        // BER history plot
        if !manager.sim_ber_history.is_empty() {
            let plot = Plot::new("ber_plot")
                .height(120.0)
                .allow_zoom(true)
                .allow_drag(true)
                .x_axis_label("Sample (x256 bits)")
                .y_axis_label("BER")
                .include_y(0.0)
                .include_y(0.5); // Max Y at 0.5 to see variations better

            plot.show(ui, |plot_ui| {
                // BER history line
                let ber_points: PlotPoints = manager
                    .sim_ber_history
                    .iter()
                    .enumerate()
                    .map(|(i, &ber)| [i as f64, ber])
                    .collect();

                plot_ui.line(
                    Line::new(ber_points)
                        .name("BER (window)")
                        .color(Color32::from_rgb(255, 100, 100))
                        .width(2.0),
                );

                // Reference lines for common BER thresholds
                let len = manager.sim_ber_history.len();
                if len > 0 {
                    // 10^-3 threshold (good for voice)
                    let threshold_1e3: PlotPoints = vec![[0.0, 0.001], [len as f64, 0.001]].into();
                    plot_ui.line(
                        Line::new(threshold_1e3)
                            .name("10⁻³ (voice)")
                            .color(Color32::from_rgba_unmultiplied(100, 255, 100, 100))
                            .style(egui_plot::LineStyle::Dashed { length: 5.0 }),
                    );

                    // 10^-6 threshold (good for data)
                    let threshold_1e6: PlotPoints = vec![[0.0, 0.000001], [len as f64, 0.000001]].into();
                    plot_ui.line(
                        Line::new(threshold_1e6)
                            .name("10⁻⁶ (data)")
                            .color(Color32::from_rgba_unmultiplied(100, 100, 255, 100))
                            .style(egui_plot::LineStyle::Dashed { length: 5.0 }),
                    );
                }
            });
        } else {
            ui.colored_label(Color32::GRAY, "Start simulation to see BER history");
        }
    }

    /// Render waterfall spectrogram
    fn render_waterfall(&mut self, ui: &mut Ui, manager: &StreamManager) {
        let history = manager.waterfall.get_history();

        if history.is_empty() {
            ui.colored_label(Color32::YELLOW, "No spectrum data. Start playback to see waterfall.");
            // Show empty space
            let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 150.0), egui::Sense::hover());
            ui.painter().rect_filled(rect, 0.0, Color32::from_gray(20));
            return;
        }

        // Waterfall parameters
        let width = ui.available_width().min(800.0);
        let height = 150.0;
        let (rect, _response) = ui.allocate_exact_size(Vec2::new(width, height), egui::Sense::hover());

        let painter = ui.painter_at(rect);

        // Draw background
        painter.rect_filled(rect, 0.0, Color32::from_gray(10));

        let rows = history.len();
        let cols = if rows > 0 { history[0].len() } else { 0 };

        if rows == 0 || cols == 0 {
            return;
        }

        let cell_height = height / rows as f32;
        let cell_width = width / cols as f32;

        // Draw each cell
        for (row_idx, row) in history.iter().enumerate() {
            let y = rect.top() + row_idx as f32 * cell_height;

            for (col_idx, &power_db) in row.iter().enumerate() {
                let x = rect.left() + col_idx as f32 * cell_width;

                // Normalize power to 0-1
                let normalized = manager.waterfall.normalize_power(power_db);

                // Get color from colormap
                let color = match self.colormap {
                    Colormap::Viridis => viridis_color(normalized),
                    Colormap::Jet => jet_color(normalized),
                };

                let cell_rect = Rect::from_min_size(
                    Pos2::new(x, y),
                    Vec2::new(cell_width.ceil(), cell_height.ceil()),
                );
                painter.rect_filled(cell_rect, 0.0, color);
            }
        }

        // Draw frequency axis labels
        let sample_rate = manager.config.sample_rate;
        ui.horizontal(|ui| {
            ui.label(format!("-{:.0} kHz", sample_rate / 2000.0));
            ui.add_space(width / 2.0 - 60.0);
            ui.label("0");
            ui.add_space(width / 2.0 - 60.0);
            ui.label(format!("+{:.0} kHz", sample_rate / 2000.0));
        });

        // dB range controls
        ui.horizontal(|ui| {
            ui.label("dB Range:");
            let mut min_db = manager.waterfall.min_db;
            let mut max_db = manager.waterfall.max_db;
            ui.add(egui::Slider::new(&mut min_db, -100.0..=0.0).text("Min"));
            ui.add(egui::Slider::new(&mut max_db, -60.0..=20.0).text("Max"));
            // Note: Can't modify manager here since it's borrowed immutably
            // Would need to restructure to update these
        });
    }

    /// Render real-time spectrum plot (power vs frequency)
    fn render_spectrum_plot(&mut self, ui: &mut Ui, manager: &StreamManager) {
        // Get current spectrum from waterfall
        let current_spectrum = match manager.waterfall.get_current_spectrum() {
            Some(s) => s,
            None => {
                // No spectrum yet, show empty plot
                let plot = Plot::new("spectrum_plot")
                    .height(120.0)
                    .allow_zoom(true)
                    .allow_drag(true)
                    .x_axis_label("Frequency (kHz)")
                    .y_axis_label("Power (dB)");
                plot.show(ui, |_| {});
                self.render_spectrum_controls(ui);
                return;
            }
        };

        let fft_size = current_spectrum.len();
        let sample_rate = manager.config.sample_rate;
        let freq_resolution = sample_rate / fft_size as f64;

        // Process spectrum based on mode
        let display_spectrum: Vec<f32> = match self.spectrum_mode {
            SpectrumMode::Normal => current_spectrum.to_vec(),
            SpectrumMode::Average => {
                // Initialize or resize accumulator if needed
                if self.spectrum_accum.len() != fft_size {
                    self.spectrum_accum = current_spectrum.to_vec();
                    self.spectrum_frame_count = 1;
                } else {
                    // Exponential moving average
                    let alpha = 1.0 / self.spectrum_avg_count as f32;
                    for (acc, &curr) in self.spectrum_accum.iter_mut().zip(current_spectrum.iter()) {
                        *acc = *acc * (1.0 - alpha) + curr * alpha;
                    }
                    self.spectrum_frame_count += 1;
                }
                self.spectrum_accum.clone()
            }
            SpectrumMode::MaxHold => {
                // Initialize or resize accumulator if needed
                if self.spectrum_accum.len() != fft_size {
                    self.spectrum_accum = current_spectrum.to_vec();
                    self.spectrum_frame_count = 1;
                } else {
                    // Keep maximum at each bin
                    for (acc, &curr) in self.spectrum_accum.iter_mut().zip(current_spectrum.iter()) {
                        *acc = acc.max(curr);
                    }
                    self.spectrum_frame_count += 1;
                }
                self.spectrum_accum.clone()
            }
        };

        // Find peak (maximum power bin) in display spectrum
        let (peak_bin, peak_power) = display_spectrum
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, &p)| (i, p))
            .unwrap_or((fft_size / 2, -100.0));

        let peak_freq_khz = ((peak_bin as f64 - fft_size as f64 / 2.0) * freq_resolution) / 1000.0;

        // Create frequency axis (FFT-shifted: -Fs/2 to +Fs/2)
        let spectrum_points: PlotPoints = display_spectrum
            .iter()
            .enumerate()
            .map(|(i, &power_db)| {
                // Convert bin index to frequency (already FFT-shifted)
                let freq_khz = ((i as f64 - fft_size as f64 / 2.0) * freq_resolution) / 1000.0;
                [freq_khz, power_db as f64]
            })
            .collect();

        let min_freq = -sample_rate / 2000.0;
        let max_freq = sample_rate / 2000.0;

        // Track cursor position for click handling
        let mut cursor_info: Option<(f64, f32)> = None;

        let plot = Plot::new("spectrum_plot")
            .height(120.0)
            .allow_zoom(true)
            .allow_drag(true)
            .x_axis_label("Frequency (kHz)")
            .y_axis_label("Power (dB)")
            .include_x(min_freq)
            .include_x(max_freq)
            .include_y(manager.waterfall.min_db as f64)
            .include_y(manager.waterfall.max_db as f64 + 10.0);

        let plot_response = plot.show(ui, |plot_ui| {
            // Draw spectrum line
            plot_ui.line(
                Line::new(spectrum_points)
                    .name("Spectrum")
                    .color(Color32::from_rgb(100, 200, 255))
                    .width(1.5),
            );

            // Draw reference lines at min/max dB
            let min_db_line: PlotPoints = vec![
                [min_freq, manager.waterfall.min_db as f64],
                [max_freq, manager.waterfall.min_db as f64],
            ].into();
            plot_ui.line(
                Line::new(min_db_line)
                    .color(Color32::from_rgba_unmultiplied(100, 100, 100, 80))
                    .style(egui_plot::LineStyle::Dashed { length: 3.0 }),
            );

            // Zero frequency marker
            let dc_line: PlotPoints = vec![
                [0.0, manager.waterfall.min_db as f64],
                [0.0, manager.waterfall.max_db as f64 + 10.0],
            ].into();
            plot_ui.line(
                Line::new(dc_line)
                    .color(Color32::from_rgba_unmultiplied(150, 150, 150, 60))
                    .style(egui_plot::LineStyle::Dashed { length: 5.0 }),
            );

            // Peak marker
            let peak_point: PlotPoints = vec![[peak_freq_khz, peak_power as f64]].into();
            plot_ui.points(
                Points::new(peak_point)
                    .name(format!("{:.3} kHz, {:.1} dB", peak_freq_khz, peak_power))
                    .color(Color32::from_rgb(255, 100, 100))
                    .radius(5.0)
                    .shape(egui_plot::MarkerShape::Diamond),
            );

            // Peak annotation text
            plot_ui.text(
                egui_plot::Text::new(
                    egui_plot::PlotPoint::new(peak_freq_khz, peak_power as f64 + 5.0),
                    format!("{:.3} kHz\n{:.1} dB", peak_freq_khz, peak_power),
                )
                .color(Color32::from_rgb(255, 150, 150))
                .anchor(egui::Align2::CENTER_BOTTOM),
            );

            // Draw placed markers (M1 = green, M2 = magenta)
            if let Some((m1_freq, m1_power)) = self.spectrum_marker1 {
                // Marker 1 vertical line
                let m1_line: PlotPoints = vec![
                    [m1_freq, manager.waterfall.min_db as f64],
                    [m1_freq, manager.waterfall.max_db as f64 + 10.0],
                ].into();
                plot_ui.line(
                    Line::new(m1_line)
                        .color(Color32::from_rgba_unmultiplied(100, 255, 100, 200))
                        .width(2.0),
                );
                // Marker 1 point
                let m1_point: PlotPoints = vec![[m1_freq, m1_power as f64]].into();
                plot_ui.points(
                    Points::new(m1_point)
                        .name("M1")
                        .color(Color32::from_rgb(100, 255, 100))
                        .radius(6.0)
                        .shape(egui_plot::MarkerShape::Square),
                );
                plot_ui.text(
                    egui_plot::Text::new(
                        egui_plot::PlotPoint::new(m1_freq, m1_power as f64 + 4.0),
                        format!("M1: {:.3} kHz\n{:.1} dB", m1_freq, m1_power),
                    )
                    .color(Color32::from_rgb(100, 255, 100))
                    .anchor(egui::Align2::CENTER_BOTTOM),
                );
            }

            if let Some((m2_freq, m2_power)) = self.spectrum_marker2 {
                // Marker 2 vertical line
                let m2_line: PlotPoints = vec![
                    [m2_freq, manager.waterfall.min_db as f64],
                    [m2_freq, manager.waterfall.max_db as f64 + 10.0],
                ].into();
                plot_ui.line(
                    Line::new(m2_line)
                        .color(Color32::from_rgba_unmultiplied(255, 100, 255, 200))
                        .width(2.0),
                );
                // Marker 2 point
                let m2_point: PlotPoints = vec![[m2_freq, m2_power as f64]].into();
                plot_ui.points(
                    Points::new(m2_point)
                        .name("M2")
                        .color(Color32::from_rgb(255, 100, 255))
                        .radius(6.0)
                        .shape(egui_plot::MarkerShape::Square),
                );
                plot_ui.text(
                    egui_plot::Text::new(
                        egui_plot::PlotPoint::new(m2_freq, m2_power as f64 + 4.0),
                        format!("M2: {:.3} kHz\n{:.1} dB", m2_freq, m2_power),
                    )
                    .color(Color32::from_rgb(255, 100, 255))
                    .anchor(egui::Align2::CENTER_BOTTOM),
                );
            }

            // Draw bandwidth region between markers if both are set
            if let (Some((m1_freq, m1_power)), Some((m2_freq, m2_power))) =
                (self.spectrum_marker1, self.spectrum_marker2)
            {
                let left_freq = m1_freq.min(m2_freq);
                let right_freq = m1_freq.max(m2_freq);
                let mid_freq = (m1_freq + m2_freq) / 2.0;
                let mid_power = ((m1_power + m2_power) / 2.0) as f64;

                // Horizontal line connecting markers
                let bw_line: PlotPoints = vec![
                    [m1_freq, m1_power as f64],
                    [m2_freq, m2_power as f64],
                ].into();
                plot_ui.line(
                    Line::new(bw_line)
                        .color(Color32::from_rgba_unmultiplied(200, 200, 100, 150))
                        .width(1.5)
                        .style(egui_plot::LineStyle::Dashed { length: 4.0 }),
                );

                // Bandwidth annotation
                let bw_khz = (right_freq - left_freq).abs();
                let delta_db = m2_power - m1_power;
                plot_ui.text(
                    egui_plot::Text::new(
                        egui_plot::PlotPoint::new(mid_freq, mid_power - 6.0),
                        format!("BW: {:.3} kHz\nΔdB: {:.1}", bw_khz, delta_db),
                    )
                    .color(Color32::from_rgb(200, 200, 100))
                    .anchor(egui::Align2::CENTER_TOP),
                );
            }

            // Cursor measurement - get pointer position in plot coordinates
            if let Some(pointer_pos) = plot_ui.pointer_coordinate() {
                let cursor_freq_khz = pointer_pos.x;

                // Find the spectrum value at this frequency
                // Convert frequency to bin index
                let bin_index = ((cursor_freq_khz * 1000.0 / freq_resolution) + (fft_size as f64 / 2.0)) as usize;

                if bin_index < display_spectrum.len() {
                    let cursor_power = display_spectrum[bin_index];

                    // Draw vertical cursor line
                    let v_line: PlotPoints = vec![
                        [cursor_freq_khz, manager.waterfall.min_db as f64],
                        [cursor_freq_khz, manager.waterfall.max_db as f64 + 10.0],
                    ].into();
                    plot_ui.line(
                        Line::new(v_line)
                            .color(Color32::from_rgba_unmultiplied(255, 255, 100, 150))
                            .width(1.0),
                    );

                    // Draw horizontal cursor line at spectrum value
                    let h_line: PlotPoints = vec![
                        [min_freq, cursor_power as f64],
                        [max_freq, cursor_power as f64],
                    ].into();
                    plot_ui.line(
                        Line::new(h_line)
                            .color(Color32::from_rgba_unmultiplied(255, 255, 100, 100))
                            .width(1.0),
                    );

                    // Cursor point marker
                    let cursor_point: PlotPoints = vec![[cursor_freq_khz, cursor_power as f64]].into();
                    plot_ui.points(
                        Points::new(cursor_point)
                            .color(Color32::from_rgb(255, 255, 100))
                            .radius(4.0)
                            .shape(egui_plot::MarkerShape::Circle),
                    );

                    // Delta from peak
                    let delta_freq = cursor_freq_khz - peak_freq_khz;
                    let delta_power = cursor_power - peak_power;

                    // Cursor annotation
                    plot_ui.text(
                        egui_plot::Text::new(
                            egui_plot::PlotPoint::new(cursor_freq_khz, cursor_power as f64 - 3.0),
                            format!("{:.3} kHz, {:.1} dB\nΔpeak: {:.3} kHz, {:.1} dB",
                                cursor_freq_khz, cursor_power, delta_freq, delta_power),
                        )
                        .color(Color32::from_rgb(255, 255, 150))
                        .anchor(egui::Align2::CENTER_TOP),
                    );

                    // Store cursor position for click handling
                    cursor_info = Some((cursor_freq_khz, cursor_power));
                }
            }
        });

        // Handle clicks to place markers (outside the plot closure)
        if plot_response.response.clicked() {
            if let Some((freq, power)) = cursor_info {
                // Place marker: if M1 is empty, set M1; else if M2 is empty, set M2; else reset and set M1
                if self.spectrum_marker1.is_none() {
                    self.spectrum_marker1 = Some((freq, power));
                } else if self.spectrum_marker2.is_none() {
                    self.spectrum_marker2 = Some((freq, power));
                } else {
                    // Both set, start over
                    self.spectrum_marker1 = Some((freq, power));
                    self.spectrum_marker2 = None;
                }
            }
        }

        // Spectrum mode controls
        self.render_spectrum_controls(ui);
    }

    /// Render spectrum mode controls
    fn render_spectrum_controls(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.label("Mode:");
            egui::ComboBox::from_id_salt("spectrum_mode")
                .selected_text(match self.spectrum_mode {
                    SpectrumMode::Normal => "Normal",
                    SpectrumMode::Average => "Average",
                    SpectrumMode::MaxHold => "Max Hold",
                })
                .show_ui(ui, |ui| {
                    if ui.selectable_value(&mut self.spectrum_mode, SpectrumMode::Normal, "Normal").changed() {
                        self.spectrum_accum.clear();
                        self.spectrum_frame_count = 0;
                    }
                    if ui.selectable_value(&mut self.spectrum_mode, SpectrumMode::Average, "Average").changed() {
                        self.spectrum_accum.clear();
                        self.spectrum_frame_count = 0;
                    }
                    if ui.selectable_value(&mut self.spectrum_mode, SpectrumMode::MaxHold, "Max Hold").changed() {
                        self.spectrum_accum.clear();
                        self.spectrum_frame_count = 0;
                    }
                });

            // Show averaging count control when in Average mode
            if self.spectrum_mode == SpectrumMode::Average {
                ui.separator();
                ui.label("Avg:");
                let mut avg = self.spectrum_avg_count as u32;
                if ui.add(egui::Slider::new(&mut avg, 2..=100).logarithmic(true)).changed() {
                    self.spectrum_avg_count = avg as usize;
                }
            }

            // Show frame count for Average/MaxHold modes
            if self.spectrum_mode != SpectrumMode::Normal {
                ui.separator();
                ui.label(format!("Frames: {}", self.spectrum_frame_count));

                // Reset button
                if ui.button("Reset").clicked() {
                    self.spectrum_accum.clear();
                    self.spectrum_frame_count = 0;
                }
            }

            ui.separator();

            // Marker controls
            let has_markers = self.spectrum_marker1.is_some() || self.spectrum_marker2.is_some();
            if has_markers {
                // Show marker info
                if let Some((m1_freq, m1_power)) = self.spectrum_marker1 {
                    ui.colored_label(Color32::from_rgb(100, 255, 100),
                        format!("M1: {:.2}kHz {:.1}dB", m1_freq, m1_power));
                }
                if let Some((m2_freq, m2_power)) = self.spectrum_marker2 {
                    ui.colored_label(Color32::from_rgb(255, 100, 255),
                        format!("M2: {:.2}kHz {:.1}dB", m2_freq, m2_power));
                }
                // Show bandwidth if both markers set
                if let (Some((m1_freq, _)), Some((m2_freq, _))) =
                    (self.spectrum_marker1, self.spectrum_marker2)
                {
                    let bw = (m2_freq - m1_freq).abs();
                    ui.colored_label(Color32::from_rgb(200, 200, 100),
                        format!("BW: {:.3}kHz", bw));
                }
                // Clear markers button
                if ui.button("Clear Markers").clicked() {
                    self.spectrum_marker1 = None;
                    self.spectrum_marker2 = None;
                }
            } else {
                ui.colored_label(Color32::GRAY, "Click to place markers");
            }
        });
    }

    /// Render real-time statistics
    fn render_statistics(&self, ui: &mut Ui, manager: &StreamManager) {
        ui.horizontal(|ui| {
            ui.label(format!("Avg Power: {:.1} dB", manager.stats.average_power_db));
            ui.separator();
            ui.label(format!("Peak: {:.1} dB", manager.stats.peak_power_db));
            ui.separator();
            ui.label(format!("Sample Rate: {:.0} kHz", manager.config.sample_rate / 1000.0));
            ui.separator();
            ui.label(format!("Buffer: {} / {}",
                manager.time_buffer.len(),
                manager.time_buffer.capacity()));
            ui.separator();
            ui.label(format!("Waterfall: {} rows", manager.waterfall.depth()));
        });

        // UDP-specific statistics (native only)
        #[cfg(not(target_arch = "wasm32"))]
        if let StreamSource::Udp { port, format, status, packets_received, samples_received } = &manager.source {
            ui.horizontal(|ui| {
                // Status indicator with color
                let (status_text, status_color) = match status {
                    UdpStatus::Disconnected => ("Disconnected", Color32::GRAY),
                    UdpStatus::Listening => ("Listening", Color32::YELLOW),
                    UdpStatus::Receiving => ("Receiving", Color32::GREEN),
                    UdpStatus::Error => ("Error", Color32::RED),
                };
                ui.colored_label(status_color, format!("UDP: {}", status_text));
                ui.separator();
                ui.label(format!("Port: {}", port));
                ui.separator();
                ui.label(format!("Format: {}", format.name()));
                ui.separator();
                ui.label(format!("Packets: {}", packets_received));
                ui.separator();
                ui.label(format!("Samples: {}", samples_received));
            });
        }

        // Simulation-specific statistics
        if manager.is_simulation() {
            ui.horizontal(|ui| {
                // BER display with color coding
                let ber_color = if manager.sim_ber < 0.001 {
                    Color32::GREEN
                } else if manager.sim_ber < 0.01 {
                    Color32::YELLOW
                } else {
                    Color32::RED
                };
                ui.colored_label(ber_color, format!("BER: {:.2e}", manager.sim_ber));
                ui.separator();
                ui.label(format!("TX bits: {}", manager.sim_tx_bits.len()));
                ui.separator();
                ui.label(format!("RX bits: {}", manager.sim_demod_bits.len()));
                ui.separator();
                ui.label(format!("SNR: {:.1} dB", manager.sim_snr_db));
                ui.separator();
                ui.label(format!("Channel: {:?}", manager.sim_channel_model));
                if manager.sim_cfo_hz.abs() > 0.1 {
                    ui.separator();
                    ui.label(format!("CFO: {:.0} Hz", manager.sim_cfo_hz));
                }
                if manager.sim_phase_noise_deg > 0.001 {
                    ui.separator();
                    ui.label(format!("PN: {:.2}°", manager.sim_phase_noise_deg));
                }
                let has_iq_imbalance = manager.sim_iq_gain_db.abs() > 0.001 || manager.sim_iq_phase_deg.abs() > 0.01;
                if has_iq_imbalance {
                    ui.separator();
                    ui.label(format!("IQ: {:.2}dB/{:.1}°", manager.sim_iq_gain_db, manager.sim_iq_phase_deg));
                }
                let has_dc_offset = manager.sim_dc_offset_i.abs() > 1e-6 || manager.sim_dc_offset_q.abs() > 1e-6;
                if has_dc_offset {
                    ui.separator();
                    ui.label(format!("DC: I={:.3}/Q={:.3}", manager.sim_dc_offset_i, manager.sim_dc_offset_q));
                }
            });
        }
    }
}
