//! Generic Pipeline View
//!
//! Displays the complete modulation → channel → demodulation pipeline for any waveform.

use egui::{Color32, RichText, Ui};
use egui_plot::{Line, Plot, PlotPoints, Points};
use r4w_core::types::IQSample;
use r4w_core::waveform::{DemodulationStep, ModulationStage, Waveform};

/// Generic full pipeline view that works with any waveform
pub struct GenericPipelineView {
    /// Show I component in plots
    show_i: bool,
    /// Show Q component in plots
    show_q: bool,
    /// Channel noise level (0.0 = no noise)
    noise_level: f32,
    /// Cached modulation stages
    cached_mod_stages: Vec<ModulationStage>,
    /// Cached demodulation steps
    cached_demod_steps: Vec<DemodulationStep>,
    /// Cached clean modulated samples (before channel)
    cached_clean_samples: Option<Vec<IQSample>>,
    /// Cached noisy samples (after channel)
    cached_noisy_samples: Option<Vec<IQSample>>,
    /// Last waveform name
    last_waveform: String,
    /// Last test data hash (to detect changes)
    last_data_hash: u64,
    /// Last noise level (to detect noise changes)
    last_noise_level: f32,
}

impl Default for GenericPipelineView {
    fn default() -> Self {
        Self::new()
    }
}

impl GenericPipelineView {
    pub fn new() -> Self {
        Self {
            show_i: true,
            show_q: true,
            noise_level: 0.0,
            cached_mod_stages: Vec::new(),
            cached_demod_steps: Vec::new(),
            cached_clean_samples: None,
            cached_noisy_samples: None,
            last_waveform: String::new(),
            last_data_hash: 0,
            last_noise_level: 0.0,
        }
    }

    /// Render the full pipeline view
    pub fn render(
        &mut self,
        ui: &mut Ui,
        waveform: &dyn Waveform,
        test_data: &[u8],
        samples: &Option<Vec<IQSample>>,
    ) {
        let info = waveform.info();

        ui.heading(format!("{} Complete Pipeline", info.name));
        ui.add_space(4.0);

        ui.label(format!(
            "{} - Full Modulation → Channel → Demodulation",
            info.full_name
        ));

        ui.add_space(8.0);

        // Controls
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.show_i, "I");
            ui.checkbox(&mut self.show_q, "Q");
            ui.separator();
            ui.label("Channel Noise:");
            ui.add(egui::Slider::new(&mut self.noise_level, 0.0..=0.5).suffix("σ"));
        });

        ui.add_space(8.0);

        // Calculate data hash for change detection
        let data_hash = test_data.iter().fold(0u64, |acc, &b| acc.wrapping_mul(31).wrapping_add(b as u64));

        // Update caches if needed (waveform, data, or noise changed)
        if self.last_waveform != info.name || self.last_data_hash != data_hash || (self.last_noise_level - self.noise_level).abs() > 0.001 {
            self.update_cache(waveform, test_data, samples);
            self.last_waveform = info.name.to_string();
            self.last_data_hash = data_hash;
            self.last_noise_level = self.noise_level;
        }

        // Pipeline diagram
        self.render_pipeline_overview(ui, &info.name);

        ui.add_space(12.0);
        ui.separator();

        // Three-column layout: Modulation | Channel | Demodulation
        ui.horizontal(|ui| {
            // Modulation column
            ui.group(|ui| {
                ui.set_min_width(280.0);
                ui.vertical(|ui| {
                    ui.heading(RichText::new("📡 Modulation").color(Color32::LIGHT_BLUE));
                    self.render_modulation_summary(ui, test_data);
                });
            });

            // Channel column
            ui.group(|ui| {
                ui.set_min_width(280.0);
                ui.vertical(|ui| {
                    ui.heading(RichText::new("📶 Channel").color(Color32::LIGHT_GREEN));
                    self.render_channel_summary(ui, samples);
                });
            });

            // Demodulation column
            ui.group(|ui| {
                ui.set_min_width(280.0);
                ui.vertical(|ui| {
                    ui.heading(RichText::new("📥 Demodulation").color(Color32::from_rgb(150, 200, 255)));
                    self.render_demodulation_summary(ui, test_data);
                });
            });
        });

        ui.add_space(12.0);
        ui.separator();

        // Signal comparison
        self.render_signal_comparison(ui, samples);

        ui.add_space(12.0);

        // Bit error analysis
        self.render_bit_error_analysis(ui, test_data);
    }

    fn update_cache(
        &mut self,
        waveform: &dyn Waveform,
        test_data: &[u8],
        _samples: &Option<Vec<IQSample>>,
    ) {
        // Get modulation stages
        self.cached_mod_stages = waveform.get_modulation_stages(test_data);

        // Generate samples using the waveform's own modulate() function
        // This ensures modulation and demodulation are consistent
        let modulated_samples = waveform.modulate(test_data);

        if modulated_samples.is_empty() {
            self.cached_clean_samples = None;
            self.cached_noisy_samples = None;
            self.cached_demod_steps = Vec::new();
            return;
        }

        // Store clean samples for display
        self.cached_clean_samples = Some(modulated_samples.clone());

        // Apply channel noise
        self.cached_noisy_samples = Some(self.apply_noise(&modulated_samples));

        // Get demodulation steps using our own modulated samples
        if let Some(ref noisy) = self.cached_noisy_samples {
            self.cached_demod_steps = waveform.get_demodulation_steps(noisy);
        } else {
            self.cached_demod_steps = waveform.get_demodulation_steps(&modulated_samples);
        }
    }

    fn apply_noise(&self, samples: &[IQSample]) -> Vec<IQSample> {
        if self.noise_level <= 0.0 {
            return samples.to_vec();
        }

        // Simple AWGN simulation
        samples
            .iter()
            .enumerate()
            .map(|(i, s)| {
                // Deterministic pseudo-random noise based on index
                let noise_i = (i as f64 * 1.618033988749).sin() * self.noise_level as f64;
                let noise_q = (i as f64 * 2.718281828459).cos() * self.noise_level as f64;
                IQSample::new(s.re + noise_i, s.im + noise_q)
            })
            .collect()
    }

    fn render_pipeline_overview(&self, ui: &mut Ui, waveform_name: &str) {
        ui.collapsing("Full Pipeline Diagram", |ui| {
            let diagram = format!(r#"
┌───────────────────────────────────────────────────────────────────────────────────┐
│                         {} Complete Pipeline                                      │
├───────────────────────────────────────────────────────────────────────────────────┤
│                                                                                   │
│  ┌──────────┐    ┌────────────┐    ┌──────────┐    ┌────────────┐    ┌──────────┐ │
│  │  Input   │───►│ Modulation │───►│ Channel  │───►│Demodulation│───►│  Output  │ │
│  │  Bits    │    │  Pipeline  │    │  Model   │    │  Pipeline  │    │  Bits    │ │
│  └──────────┘    └────────────┘    └──────────┘    └────────────┘    └──────────┘ │
│       │                │                │                │                │       │
│       ▼                ▼                ▼                ▼                ▼       │
│   Original       I/Q Samples         Noise         Recovered        Compare       │
│    Data          (Baseband)         Addition         Data          (BER calc)     │
│                                                                                   │
└───────────────────────────────────────────────────────────────────────────────────┘"#, waveform_name);
            ui.code(&diagram);
        });
    }

    fn render_modulation_summary(&self, ui: &mut Ui, test_data: &[u8]) {
        ui.label(format!("Input: {} bytes", test_data.len()));

        // Show input preview
        let hex: String = test_data
            .iter()
            .take(8)
            .map(|b| format!("{:02X}", b))
            .collect::<Vec<_>>()
            .join(" ");
        ui.small(format!("Hex: {}", hex));

        ui.add_space(4.0);

        // List stages
        for (i, stage) in self.cached_mod_stages.iter().enumerate() {
            ui.horizontal(|ui| {
                ui.label(format!("{}.", i + 1));
                ui.label(&stage.name);
            });
        }

        // Show final sample count
        if let Some(last) = self.cached_mod_stages.last() {
            if let Some(ref samples) = last.samples {
                ui.add_space(4.0);
                ui.label(format!("Output: {} samples", samples.len()));
            }
        }
    }

    fn render_channel_summary(&self, ui: &mut Ui, _samples: &Option<Vec<IQSample>>) {
        ui.label("Model: AWGN");
        ui.label(format!("Noise σ: {:.3}", self.noise_level));

        if let Some(ref signal) = self.cached_clean_samples {
            let power: f64 = signal.iter().map(|s| s.re * s.re + s.im * s.im).sum::<f64>() / signal.len() as f64;
            let snr_db = if self.noise_level > 0.0 {
                10.0 * (power / (self.noise_level as f64 * self.noise_level as f64)).log10()
            } else {
                f64::INFINITY
            };

            ui.add_space(4.0);
            ui.label(format!("Signal Power: {:.3}", power));
            if snr_db.is_finite() {
                ui.label(format!("Approx SNR: {:.1} dB", snr_db));
            } else {
                ui.label("SNR: ∞ (no noise)");
            }
        }

        if let Some(ref noisy) = self.cached_noisy_samples {
            ui.add_space(4.0);
            ui.label(format!("Samples: {}", noisy.len()));
        }
    }

    fn render_demodulation_summary(&self, ui: &mut Ui, original_data: &[u8]) {
        // List steps
        for (i, step) in self.cached_demod_steps.iter().enumerate() {
            ui.horizontal(|ui| {
                ui.label(format!("{}.", i + 1));
                ui.label(&step.name);
            });
        }

        // Show recovered data
        if let Some(last) = self.cached_demod_steps.last() {
            if let Some(ref bits) = last.recovered_bits {
                // Pack bits to bytes if needed for display
                let display_bytes = if Self::is_individual_bits(bits) {
                    Self::pack_bits_to_bytes(bits)
                } else {
                    bits.clone()
                };

                // Check if recovered matches original
                let min_len = original_data.len().min(display_bytes.len());
                let content_matches = min_len > 0 &&
                    (0..min_len).all(|i| original_data[i] == display_bytes[i]);
                let length_matches = display_bytes.len() == original_data.len();

                // Color logic:
                // - Green: Perfect match (same length and content)
                // - Orange: Content matches but length differs (symbol padding)
                // - Red: Actual bit errors in compared content
                let color = if content_matches && length_matches {
                    Color32::GREEN
                } else if content_matches {
                    Color32::from_rgb(255, 165, 0) // Orange - padding but no errors
                } else {
                    Color32::RED
                };

                ui.add_space(4.0);

                // Show recovered bytes count with appropriate color
                ui.colored_label(color, format!("Recovered: {} bytes", display_bytes.len()));

                let hex: String = display_bytes
                    .iter()
                    .take(8)
                    .map(|b| format!("{:02X}", b))
                    .collect::<Vec<_>>()
                    .join(" ");

                ui.small(RichText::new(format!("Hex: {}", hex)).color(color));
            }
        }
    }

    fn render_signal_comparison(&mut self, ui: &mut Ui, _samples: &Option<Vec<IQSample>>) {
        ui.heading("Signal Comparison");

        if self.cached_clean_samples.is_none() {
            ui.label("Generate a signal to see comparison.");
            return;
        }

        ui.horizontal(|ui| {
            // Original signal
            ui.group(|ui| {
                ui.set_min_width(400.0);
                ui.label("Original Signal (Tx)");

                if let Some(ref signal) = self.cached_clean_samples {
                    let plot = Plot::new("pipeline_tx_signal")
                        .height(150.0)
                        .allow_zoom(true)
                        .allow_drag(true);

                    let display_len = signal.len().min(500);

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
                }
            });

            // Received signal (with noise)
            ui.group(|ui| {
                ui.set_min_width(400.0);
                ui.label("Received Signal (Rx + Noise)");

                if let Some(ref noisy) = self.cached_noisy_samples {
                    let plot = Plot::new("pipeline_rx_signal")
                        .height(150.0)
                        .allow_zoom(true)
                        .allow_drag(true);

                    let display_len = noisy.len().min(500);

                    plot.show(ui, |plot_ui| {
                        if self.show_i {
                            let i_points: PlotPoints = (0..display_len)
                                .map(|i| [i as f64, noisy[i].re])
                                .collect();
                            plot_ui.line(
                                Line::new(i_points)
                                    .name("I")
                                    .color(Color32::BLUE),
                            );
                        }

                        if self.show_q {
                            let q_points: PlotPoints = (0..display_len)
                                .map(|i| [i as f64, noisy[i].im])
                                .collect();
                            plot_ui.line(
                                Line::new(q_points)
                                    .name("Q")
                                    .color(Color32::RED),
                            );
                        }
                    });
                }
            });
        });

        // Constellation comparison
        ui.add_space(8.0);
        ui.horizontal(|ui| {
            // Tx constellation
            ui.group(|ui| {
                ui.set_min_width(200.0);
                ui.label("Tx Constellation");

                if let Some(ref signal) = self.cached_clean_samples {
                    let plot = Plot::new("pipeline_tx_const")
                        .height(180.0)
                        .width(180.0)
                        .data_aspect(1.0);

                    let display_len = signal.len().min(500);
                    let step = if display_len > 100 { display_len / 100 } else { 1 };

                    plot.show(ui, |plot_ui| {
                        let points: PlotPoints = (0..display_len)
                            .step_by(step)
                            .map(|i| [signal[i].re, signal[i].im])
                            .collect();
                        plot_ui.points(
                            Points::new(points)
                                .radius(3.0)
                                .color(Color32::from_rgb(100, 200, 100)),
                        );
                    });
                }
            });

            // Rx constellation
            ui.group(|ui| {
                ui.set_min_width(200.0);
                ui.label("Rx Constellation (with noise)");

                if let Some(ref noisy) = self.cached_noisy_samples {
                    let plot = Plot::new("pipeline_rx_const")
                        .height(180.0)
                        .width(180.0)
                        .data_aspect(1.0);

                    let display_len = noisy.len().min(500);
                    let step = if display_len > 100 { display_len / 100 } else { 1 };

                    plot.show(ui, |plot_ui| {
                        let points: PlotPoints = (0..display_len)
                            .step_by(step)
                            .map(|i| [noisy[i].re, noisy[i].im])
                            .collect();
                        plot_ui.points(
                            Points::new(points)
                                .radius(3.0)
                                .color(Color32::from_rgb(180, 130, 220)),
                        );
                    });
                }
            });
        });
    }

    /// Check if data appears to be individual bits (all values 0 or 1)
    fn is_individual_bits(data: &[u8]) -> bool {
        data.len() > 1 && data.iter().all(|&b| b == 0 || b == 1)
    }

    /// Pack individual bits into bytes (MSB first)
    fn pack_bits_to_bytes(bits: &[u8]) -> Vec<u8> {
        bits.chunks(8)
            .map(|chunk| {
                chunk.iter()
                    .enumerate()
                    .fold(0u8, |acc, (i, &bit)| {
                        acc | ((bit & 1) << (7 - i))
                    })
            })
            .collect()
    }

    fn render_bit_error_analysis(&self, ui: &mut Ui, original_data: &[u8]) {
        ui.heading("Bit Error Analysis");

        // Get recovered bits from last demod step
        let recovered = self.cached_demod_steps.last()
            .and_then(|step| step.recovered_bits.as_ref());

        if let Some(recovered_bits) = recovered {
            // Check if recovered data is individual bits (0/1 values) and pack if needed
            let packed_recovered: Vec<u8>;
            let recovered_for_comparison = if Self::is_individual_bits(recovered_bits) {
                packed_recovered = Self::pack_bits_to_bytes(recovered_bits);
                &packed_recovered
            } else {
                recovered_bits
            };

            let min_len = original_data.len().min(recovered_for_comparison.len());

            if min_len == 0 {
                ui.label("No data to compare.");
                return;
            }

            // Count bit errors
            let mut bit_errors = 0;
            let mut total_bits = 0;

            for i in 0..min_len {
                let diff = original_data[i] ^ recovered_for_comparison[i];
                bit_errors += diff.count_ones() as usize;
                total_bits += 8;
            }

            let ber = bit_errors as f64 / total_bits as f64;

            ui.horizontal(|ui| {
                ui.label(format!("Original: {} bytes", original_data.len()));
                ui.separator();
                ui.label(format!("Recovered: {} bytes", recovered_for_comparison.len()));
                ui.separator();
                ui.label(format!("Compared: {} bytes", min_len));
            });

            ui.add_space(4.0);

            let (ber_color, ber_text) = if bit_errors == 0 {
                (Color32::GREEN, "Perfect")
            } else if ber < 0.001 {
                (Color32::LIGHT_GREEN, "Excellent")
            } else if ber < 0.01 {
                (Color32::YELLOW, "Good")
            } else if ber < 0.1 {
                (Color32::LIGHT_RED, "Fair")
            } else {
                (Color32::RED, "Poor")
            };

            ui.horizontal(|ui| {
                ui.label("Bit Errors:");
                ui.colored_label(ber_color, format!("{} / {} bits", bit_errors, total_bits));
            });

            ui.horizontal(|ui| {
                ui.label("BER:");
                ui.colored_label(ber_color, format!("{:.2e} ({})", ber, ber_text));
            });

            // Show first few bytes comparison
            ui.add_space(4.0);
            ui.collapsing("Byte Comparison", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Original:");
                    let hex: String = original_data
                        .iter()
                        .take(16)
                        .map(|b| format!("{:02X}", b))
                        .collect::<Vec<_>>()
                        .join(" ");
                    ui.code(&hex);
                });

                ui.horizontal(|ui| {
                    ui.label("Recovered:");
                    let hex: String = recovered_for_comparison
                        .iter()
                        .take(16)
                        .map(|b| format!("{:02X}", b))
                        .collect::<Vec<_>>()
                        .join(" ");
                    ui.code(&hex);
                });
            });
        } else {
            ui.label("No demodulated data available for comparison.");
            ui.small("Generate a signal to run the full pipeline.");
        }
    }
}
