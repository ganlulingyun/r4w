//! Waveform Performance Comparison View
//!
//! Displays benchmark results comparing different waveforms under various conditions.

use egui::{Color32, RichText, Ui};
use egui_plot::{Bar, BarChart, Legend, Plot};

/// Benchmark data for a single waveform at a specific SNR
#[derive(Clone, Debug)]
pub struct BenchmarkPoint {
    pub snr_db: f64,
    pub throughput_sps: f64,
    pub latency_us: f64,
    pub bits_decoded: u64,
    #[allow(dead_code)]
    pub symbols: u64,
    pub demod_rate_bps: f64,
}

/// Waveform benchmark data
#[derive(Clone, Debug)]
pub struct WaveformBenchmark {
    pub name: &'static str,
    pub color: Color32,
    #[allow(dead_code)]
    pub bits_per_symbol: u8,
    pub points: Vec<BenchmarkPoint>,
}

/// Waveform Comparison View
pub struct WaveformComparisonView {
    benchmarks: Vec<WaveformBenchmark>,
    selected_metric: MetricType,
    show_lora_sf: bool,
}

#[derive(Clone, Copy, PartialEq)]
enum MetricType {
    Throughput,
    Latency,
    BitsDecoded,
    DemodRate,
}

impl Default for WaveformComparisonView {
    fn default() -> Self {
        Self::new()
    }
}

impl WaveformComparisonView {
    pub fn new() -> Self {
        // Pre-populate with actual benchmark data from Pi 500 -> Pi 3 tests
        let benchmarks = vec![
            WaveformBenchmark {
                name: "BPSK",
                color: Color32::from_rgb(88, 166, 255),  // Blue
                bits_per_symbol: 1,
                points: vec![
                    BenchmarkPoint { snr_db: 20.0, throughput_sps: 83733.0, latency_us: 28.0, bits_decoded: 1296, symbols: 5898, demod_rate_bps: 162.0 },
                    BenchmarkPoint { snr_db: 10.0, throughput_sps: 81785.0, latency_us: 28.0, bits_decoded: 1266, symbols: 5763, demod_rate_bps: 158.0 },
                    BenchmarkPoint { snr_db: 5.0, throughput_sps: 86364.0, latency_us: 28.0, bits_decoded: 1336, symbols: 6081, demod_rate_bps: 167.0 },
                    BenchmarkPoint { snr_db: 0.0, throughput_sps: 78818.0, latency_us: 28.0, bits_decoded: 1221, symbols: 5556, demod_rate_bps: 152.0 },
                ],
            },
            WaveformBenchmark {
                name: "QPSK",
                color: Color32::from_rgb(63, 185, 80),   // Green
                bits_per_symbol: 2,
                points: vec![
                    BenchmarkPoint { snr_db: 20.0, throughput_sps: 79257.0, latency_us: 29.0, bits_decoded: 1861, symbols: 5583, demod_rate_bps: 232.0 },
                    BenchmarkPoint { snr_db: 10.0, throughput_sps: 80724.0, latency_us: 39.0, bits_decoded: 1895, symbols: 5685, demod_rate_bps: 237.0 },
                    BenchmarkPoint { snr_db: 5.0, throughput_sps: 81109.0, latency_us: 28.0, bits_decoded: 1904, symbols: 5712, demod_rate_bps: 238.0 },
                    BenchmarkPoint { snr_db: 0.0, throughput_sps: 80625.0, latency_us: 28.0, bits_decoded: 1892, symbols: 5676, demod_rate_bps: 236.0 },
                ],
            },
            WaveformBenchmark {
                name: "LoRa SF7",
                color: Color32::from_rgb(163, 113, 247), // Purple
                bits_per_symbol: 7,
                points: vec![
                    BenchmarkPoint { snr_db: 20.0, throughput_sps: 83790.0, latency_us: 485.0, bits_decoded: 2616, symbols: 5232, demod_rate_bps: 327.0 },
                    BenchmarkPoint { snr_db: 10.0, throughput_sps: 83848.0, latency_us: 362.0, bits_decoded: 2620, symbols: 5240, demod_rate_bps: 327.0 },
                    BenchmarkPoint { snr_db: 5.0, throughput_sps: 82641.0, latency_us: 355.0, bits_decoded: 2580, symbols: 5160, demod_rate_bps: 322.0 },
                    BenchmarkPoint { snr_db: 0.0, throughput_sps: 77740.0, latency_us: 372.0, bits_decoded: 2428, symbols: 4856, demod_rate_bps: 303.0 },
                ],
            },
        ];

        Self {
            benchmarks,
            selected_metric: MetricType::BitsDecoded,
            show_lora_sf: false,
        }
    }

    pub fn render(&mut self, ui: &mut Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Waveform Performance Comparison");
            ui.add_space(5.0);
            ui.label(RichText::new("Real benchmark data from distributed TX/RX testing (Pi 500 → Pi 3, 125 kHz)")
                .color(Color32::GRAY));
            ui.add_space(15.0);

            // Metric selector
            ui.horizontal(|ui| {
                ui.label("Compare by:");
                ui.selectable_value(&mut self.selected_metric, MetricType::BitsDecoded, "Bits Decoded");
                ui.selectable_value(&mut self.selected_metric, MetricType::Throughput, "Throughput");
                ui.selectable_value(&mut self.selected_metric, MetricType::Latency, "Latency");
                ui.selectable_value(&mut self.selected_metric, MetricType::DemodRate, "Demod Rate");
                ui.add_space(20.0);
                ui.checkbox(&mut self.show_lora_sf, "Show LoRa SF comparison");
            });
            ui.add_space(10.0);

            // Bar chart
            self.render_bar_chart(ui);
            ui.add_space(20.0);

            // Data tables
            ui.collapsing("Clean Channel Performance (No Noise)", |ui| {
                self.render_clean_channel_table(ui);
            });

            ui.collapsing("Performance Under Noise", |ui| {
                self.render_noise_table(ui);
            });

            if self.show_lora_sf {
                ui.collapsing("LoRa Spreading Factor Comparison", |ui| {
                    self.render_sf_comparison(ui);
                });
            }

            ui.collapsing("Key Insights", |ui| {
                self.render_insights(ui);
            });

            ui.collapsing("When to Use Each Waveform", |ui| {
                self.render_use_cases(ui);
            });
        });
    }

    fn render_bar_chart(&self, ui: &mut Ui) {
        let snr_levels = [20.0, 10.0, 5.0, 0.0];

        let plot = Plot::new("comparison_chart")
            .legend(Legend::default())
            .height(250.0)
            .show_axes([true, true])
            .allow_drag(false)
            .allow_zoom(false);

        plot.show(ui, |plot_ui| {
            for (wf_idx, wf) in self.benchmarks.iter().enumerate() {
                let bars: Vec<Bar> = wf.points.iter().enumerate().map(|(snr_idx, point)| {
                    let value = match self.selected_metric {
                        MetricType::Throughput => point.throughput_sps / 1000.0, // kSps
                        MetricType::Latency => point.latency_us,
                        MetricType::BitsDecoded => point.bits_decoded as f64,
                        MetricType::DemodRate => point.demod_rate_bps,
                    };
                    // Offset bars for each waveform
                    let x = snr_idx as f64 * 4.0 + wf_idx as f64;
                    Bar::new(x, value).width(0.8)
                }).collect();

                let chart = BarChart::new(bars)
                    .color(wf.color)
                    .name(wf.name);
                plot_ui.bar_chart(chart);
            }
        });

        // X-axis labels
        ui.horizontal(|ui| {
            ui.add_space(50.0);
            for snr in &snr_levels {
                ui.label(format!("{} dB", snr));
                ui.add_space(60.0);
            }
        });

        // Y-axis label
        let y_label = match self.selected_metric {
            MetricType::Throughput => "Throughput (kSps)",
            MetricType::Latency => "Latency (μs)",
            MetricType::BitsDecoded => "Bits Decoded",
            MetricType::DemodRate => "Demod Rate (bps)",
        };
        ui.label(RichText::new(y_label).color(Color32::GRAY).small());
    }

    fn render_clean_channel_table(&self, ui: &mut Ui) {
        egui::Grid::new("clean_channel_grid")
            .num_columns(6)
            .striped(true)
            .spacing([20.0, 4.0])
            .show(ui, |ui| {
                // Header
                ui.label(RichText::new("Metric").strong());
                ui.label(RichText::new("BPSK").strong().color(self.benchmarks[0].color));
                ui.label(RichText::new("QPSK").strong().color(self.benchmarks[1].color));
                ui.label(RichText::new("LoRa SF7").strong().color(self.benchmarks[2].color));
                ui.label(RichText::new("Best").strong());
                ui.end_row();

                // Throughput
                ui.label("Throughput");
                ui.label("85,771 Sps");
                ui.label("77,224 Sps");
                ui.label("83,129 Sps");
                ui.label(RichText::new("BPSK").color(self.benchmarks[0].color));
                ui.end_row();

                // Bits/symbol
                ui.label("Bits/symbol");
                ui.label("1");
                ui.label("2");
                ui.label("7");
                ui.label(RichText::new("LoRa").color(self.benchmarks[2].color));
                ui.end_row();

                // Demod rate
                ui.label("Demod rate");
                ui.label("168 bps");
                ui.label("228 bps");
                ui.label(RichText::new("322 bps").color(Color32::LIGHT_GREEN));
                ui.label(RichText::new("LoRa").color(self.benchmarks[2].color));
                ui.end_row();

                // Latency
                ui.label("Avg latency");
                ui.label(RichText::new("29 μs").color(Color32::LIGHT_GREEN));
                ui.label("38 μs");
                ui.label("395 μs");
                ui.label(RichText::new("BPSK").color(self.benchmarks[0].color));
                ui.end_row();

                // P99 latency
                ui.label("P99 latency");
                ui.label(RichText::new("56 μs").color(Color32::LIGHT_GREEN));
                ui.label("61 μs");
                ui.label("565 μs");
                ui.label(RichText::new("BPSK").color(self.benchmarks[0].color));
                ui.end_row();
            });
    }

    fn render_noise_table(&self, ui: &mut Ui) {
        ui.label(RichText::new("Bits Decoded (8-second test)").strong());
        ui.add_space(5.0);

        egui::Grid::new("noise_grid")
            .num_columns(5)
            .striped(true)
            .spacing([20.0, 4.0])
            .show(ui, |ui| {
                ui.label(RichText::new("SNR").strong());
                ui.label(RichText::new("BPSK").strong().color(self.benchmarks[0].color));
                ui.label(RichText::new("QPSK").strong().color(self.benchmarks[1].color));
                ui.label(RichText::new("LoRa SF7").strong().color(self.benchmarks[2].color));
                ui.end_row();

                for point_idx in 0..4 {
                    let snr = self.benchmarks[0].points[point_idx].snr_db;
                    ui.label(format!("{} dB", snr as i32));

                    for wf in &self.benchmarks {
                        let bits = wf.points[point_idx].bits_decoded;
                        let is_best = self.benchmarks.iter()
                            .map(|w| w.points[point_idx].bits_decoded)
                            .max() == Some(bits);

                        if is_best {
                            ui.label(RichText::new(format!("{}", bits)).color(Color32::LIGHT_GREEN));
                        } else {
                            ui.label(format!("{}", bits));
                        }
                    }
                    ui.end_row();
                }
            });
    }

    fn render_sf_comparison(&self, ui: &mut Ui) {
        ui.label("LoRa Spreading Factor determines the trade-off between range and speed:");
        ui.add_space(10.0);

        egui::Grid::new("sf_grid")
            .num_columns(4)
            .striped(true)
            .spacing([20.0, 4.0])
            .show(ui, |ui| {
                ui.label(RichText::new("Parameter").strong());
                ui.label(RichText::new("SF7").strong());
                ui.label(RichText::new("SF12").strong());
                ui.label(RichText::new("Ratio").strong());
                ui.end_row();

                ui.label("Chips/symbol");
                ui.label("128");
                ui.label("4,096");
                ui.label("32x");
                ui.end_row();

                ui.label("Symbol time @ 125kHz");
                ui.label("1.02 ms");
                ui.label("32.77 ms");
                ui.label("32x slower");
                ui.end_row();

                ui.label("Bits/symbol");
                ui.label("7");
                ui.label("12");
                ui.label("1.7x more");
                ui.end_row();

                ui.label("Data rate @ 125kHz");
                ui.label("~5.5 kbps");
                ui.label("~293 bps");
                ui.label("19x slower");
                ui.end_row();

                ui.label("Processing gain");
                ui.label("7 dB");
                ui.label("21 dB");
                ui.label("+14 dB");
                ui.end_row();

                ui.label("Sensitivity");
                ui.label("-123 dBm");
                ui.label("-137 dBm");
                ui.label("14 dB better");
                ui.end_row();

                ui.label("Typical range");
                ui.label("2-5 km");
                ui.label("10-15 km");
                ui.label("~3x farther");
                ui.end_row();
            });

        ui.add_space(10.0);
        ui.label(RichText::new("Note: SF12 shows 0 decoded symbols in short benchmarks because its symbol time (32.77 ms) is too long to complete full frames in the test duration.")
            .color(Color32::YELLOW).small());
    }

    fn render_insights(&self, ui: &mut Ui) {
        ui.add_space(5.0);

        // Throughput stability
        ui.horizontal(|ui| {
            ui.label(RichText::new("📈").size(18.0));
            ui.vertical(|ui| {
                ui.label(RichText::new("Throughput Stability").strong());
                ui.label("All waveforms maintain ~77-86k Sps even at 0 dB SNR");
            });
        });
        ui.add_space(10.0);

        // Spectral efficiency
        ui.horizontal(|ui| {
            ui.label(RichText::new("📊").size(18.0));
            ui.vertical(|ui| {
                ui.label(RichText::new("Spectral Efficiency").strong());
                ui.label("LoRa decodes ~2x more bits than BPSK, ~35% more than QPSK");
            });
        });
        ui.add_space(10.0);

        // Latency trade-off
        ui.horizontal(|ui| {
            ui.label(RichText::new("⏱️").size(18.0));
            ui.vertical(|ui| {
                ui.label(RichText::new("Latency Trade-off").strong());
                ui.label("LoRa's latency is ~13x higher (FFT vs simple phase detection)");
            });
        });
        ui.add_space(10.0);

        // Real-world note
        ui.horizontal(|ui| {
            ui.label(RichText::new("⚠️").size(18.0));
            ui.vertical(|ui| {
                ui.label(RichText::new("Real-World Note").strong().color(Color32::YELLOW));
                ui.label("In actual RF, LoRa excels at -20 dB SNR where PSK fails completely");
            });
        });
    }

    fn render_use_cases(&self, ui: &mut Ui) {
        ui.add_space(5.0);

        egui::Grid::new("use_case_grid")
            .num_columns(2)
            .spacing([20.0, 10.0])
            .show(ui, |ui| {
                ui.label(RichText::new("BPSK").strong().color(self.benchmarks[0].color));
                ui.label("Low-latency apps, simple implementations, moderate noise tolerance");
                ui.end_row();

                ui.label(RichText::new("QPSK").strong().color(self.benchmarks[1].color));
                ui.label("2x BPSK data rate, good balance of complexity and performance");
                ui.end_row();

                ui.label(RichText::new("LoRa").strong().color(self.benchmarks[2].color));
                ui.label("Long range (10-15 km), extreme noise immunity, IoT sensors");
                ui.end_row();
            });
    }
}
