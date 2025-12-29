//! UDP Benchmark View for interactive waveform benchmarking
//!
//! Provides real-time performance testing with:
//! - UDP I/Q sample reception
//! - Waveform processing through any available waveform
//! - Live metrics display (throughput, latency, quality)
//! - Export to JSON/CSV

use egui::{Color32, Ui};
use egui_plot::{Line, Plot, PlotPoints};
use r4w_core::benchmark::{
    BenchmarkMetrics, BenchmarkReceiver, BenchmarkReport, MetricsSummary, SampleFormat,
    WaveformRunner,
};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Benchmark state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BenchmarkState {
    Idle,
    Running,
    Stopped,
}

/// UDP Benchmark View
pub struct UdpBenchmarkView {
    /// UDP port to listen on
    port: u16,
    /// Sample format
    format: SampleFormat,
    /// Selected waveform
    selected_waveform: String,
    /// Available waveforms
    available_waveforms: Vec<String>,
    /// Sample rate in Hz
    sample_rate: f64,
    /// Batch size for processing
    batch_size: usize,
    /// Current benchmark state
    state: BenchmarkState,
    /// Stop signal for background thread
    stop_signal: Arc<AtomicBool>,
    /// Metrics receiver from background thread
    metrics_rx: Option<std::sync::mpsc::Receiver<MetricsSummary>>,
    /// Latest metrics summary
    latest_metrics: Option<MetricsSummary>,
    /// Throughput history for plotting
    throughput_history: Vec<f64>,
    /// Latency history for plotting
    latency_history: Vec<f64>,
    /// Max history points
    max_history: usize,
    /// Last report (for export)
    last_report: Option<BenchmarkReport>,
    /// Status message
    status_message: String,
    /// Last update time
    last_update: Instant,
}

impl Default for UdpBenchmarkView {
    fn default() -> Self {
        Self::new()
    }
}

impl UdpBenchmarkView {
    pub fn new() -> Self {
        let available_waveforms: Vec<String> = WaveformRunner::available_waveforms()
            .into_iter()
            .map(|s| s.to_string())
            .collect();

        let selected = available_waveforms.first().cloned().unwrap_or_default();

        Self {
            port: 5000,
            format: SampleFormat::Float32,
            selected_waveform: selected,
            available_waveforms,
            sample_rate: 48000.0,
            batch_size: 1024,
            state: BenchmarkState::Idle,
            stop_signal: Arc::new(AtomicBool::new(false)),
            metrics_rx: None,
            latest_metrics: None,
            throughput_history: Vec::new(),
            latency_history: Vec::new(),
            max_history: 120, // 2 minutes at 1Hz updates
            last_report: None,
            status_message: "Ready".to_string(),
            last_update: Instant::now(),
        }
    }

    /// Render the benchmark view
    pub fn render(&mut self, ui: &mut Ui) {
        // Check for new metrics from background thread
        self.poll_metrics();

        ui.heading("UDP Waveform Benchmark");
        ui.add_space(4.0);

        // Configuration panel
        self.render_config_panel(ui);
        ui.add_space(8.0);

        // Control buttons
        self.render_controls(ui);
        ui.add_space(8.0);

        ui.separator();
        ui.add_space(8.0);

        // Metrics display
        if self.latest_metrics.is_some() || self.state == BenchmarkState::Running {
            self.render_metrics(ui);
            ui.add_space(8.0);

            // Plots
            self.render_plots(ui);
            ui.add_space(8.0);

            // Export options
            self.render_export(ui);
        } else {
            ui.colored_label(
                Color32::GRAY,
                "Start benchmark to see metrics. Send I/Q samples to the configured UDP port.",
            );
            ui.add_space(8.0);

            // Show usage instructions
            ui.group(|ui| {
                ui.heading("Usage");
                ui.label("1. Configure port, format, and waveform above");
                ui.label("2. Click 'Start Benchmark' to begin listening");
                ui.label("3. Send I/Q samples to the UDP port using:");
                ui.add_space(4.0);
                ui.code(format!(
                    "r4w udp-send -w {} -t 127.0.0.1:{} -m \"Test\"",
                    self.selected_waveform, self.port
                ));
                ui.add_space(4.0);
                ui.label("4. Metrics will appear in real-time as data is received");
            });
        }
    }

    /// Render configuration panel
    fn render_config_panel(&mut self, ui: &mut Ui) {
        let enabled = self.state == BenchmarkState::Idle;

        ui.horizontal(|ui| {
            ui.label("UDP Port:");
            ui.add_enabled(
                enabled,
                egui::DragValue::new(&mut self.port)
                    .range(1024..=65535)
                    .speed(1.0),
            );

            ui.separator();

            ui.label("Format:");
            ui.add_enabled_ui(enabled, |ui| {
                egui::ComboBox::from_id_salt("benchmark_format")
                    .selected_text(self.format.name())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.format, SampleFormat::Float32, "f32");
                        ui.selectable_value(&mut self.format, SampleFormat::Int16, "i16");
                    });
            });

            ui.separator();

            ui.label("Waveform:");
            ui.add_enabled_ui(enabled, |ui| {
                egui::ComboBox::from_id_salt("benchmark_waveform")
                    .selected_text(&self.selected_waveform)
                    .show_ui(ui, |ui| {
                        for wf in &self.available_waveforms {
                            ui.selectable_value(&mut self.selected_waveform, wf.clone(), wf);
                        }
                    });
            });
        });

        ui.horizontal(|ui| {
            ui.label("Sample Rate:");
            ui.add_enabled(
                enabled,
                egui::DragValue::new(&mut self.sample_rate)
                    .range(1000.0..=10_000_000.0)
                    .speed(1000.0)
                    .suffix(" Hz"),
            );

            ui.separator();

            ui.label("Batch Size:");
            ui.add_enabled(
                enabled,
                egui::DragValue::new(&mut self.batch_size)
                    .range(64..=65536)
                    .speed(64.0)
                    .suffix(" samples"),
            );
        });
    }

    /// Render control buttons
    fn render_controls(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            match self.state {
                BenchmarkState::Idle => {
                    if ui.button("Start Benchmark").clicked() {
                        self.start_benchmark();
                    }
                }
                BenchmarkState::Running => {
                    let stop_btn = egui::Button::new("Stop Benchmark")
                        .fill(Color32::from_rgb(180, 60, 60));
                    if ui.add(stop_btn).clicked() {
                        self.stop_benchmark();
                    }
                }
                BenchmarkState::Stopped => {
                    if ui.button("Start Benchmark").clicked() {
                        self.start_benchmark();
                    }
                    if ui.button("Clear Results").clicked() {
                        self.clear_results();
                    }
                }
            }

            ui.separator();

            // Status indicator
            let (status_text, status_color) = match self.state {
                BenchmarkState::Idle => ("Idle", Color32::GRAY),
                BenchmarkState::Running => ("Running", Color32::GREEN),
                BenchmarkState::Stopped => ("Stopped", Color32::YELLOW),
            };
            ui.colored_label(status_color, status_text);

            ui.separator();
            ui.label(&self.status_message);
        });
    }

    /// Render live metrics
    fn render_metrics(&mut self, ui: &mut Ui) {
        if let Some(ref metrics) = self.latest_metrics {
            ui.columns(4, |cols| {
                // Throughput
                cols[0].group(|ui| {
                    ui.heading("Throughput");
                    ui.label(format!("{:.0} Sps", metrics.throughput_samples_per_sec));
                    ui.label(format!("{:.2} Mbps", metrics.throughput_mbps));
                    ui.label(format!("{:.0} bps (data)", metrics.data_rate_bps));
                });

                // Latency
                cols[1].group(|ui| {
                    ui.heading("Latency");
                    ui.label(format!("Avg: {:.1} µs", metrics.avg_latency_us));
                    ui.label(format!("P99: {:.1} µs", metrics.p99_latency_us));
                    ui.label(format!(
                        "Min/Max: {:.1}/{:.1} µs",
                        metrics.min_latency_us, metrics.max_latency_us
                    ));
                });

                // Quality
                cols[2].group(|ui| {
                    ui.heading("Quality");
                    ui.label(format!("Bits: {}", metrics.bits_demodulated));
                    ui.label(format!("Symbols: {}", metrics.symbols_detected));
                });

                // Errors & Duration
                cols[3].group(|ui| {
                    ui.heading("Status");
                    ui.label(format!("Duration: {}", metrics.elapsed_formatted()));
                    ui.label(format!("Rx Errors: {}", metrics.receive_errors));
                    ui.label(format!("Proc Errors: {}", metrics.processing_errors));
                });
            });
        }
    }

    /// Render throughput and latency plots
    fn render_plots(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            // Throughput history plot
            ui.vertical(|ui| {
                ui.heading("Throughput (Sps)");

                let plot = Plot::new("throughput_plot")
                    .height(150.0)
                    .width(ui.available_width() / 2.0 - 10.0)
                    .allow_zoom(true)
                    .allow_drag(true)
                    .x_axis_label("Time (s)")
                    .y_axis_label("Samples/sec");

                plot.show(ui, |plot_ui| {
                    if !self.throughput_history.is_empty() {
                        let points: PlotPoints = self
                            .throughput_history
                            .iter()
                            .enumerate()
                            .map(|(i, &v)| [i as f64, v])
                            .collect();

                        plot_ui.line(
                            Line::new(points)
                                .color(Color32::from_rgb(100, 200, 255))
                                .width(2.0),
                        );
                    }
                });
            });

            ui.separator();

            // Latency histogram
            ui.vertical(|ui| {
                ui.heading("Latency Distribution");

                let plot = Plot::new("latency_plot")
                    .height(150.0)
                    .width(ui.available_width() - 10.0)
                    .allow_zoom(true)
                    .allow_drag(true)
                    .x_axis_label("Time (s)")
                    .y_axis_label("Latency (µs)");

                plot.show(ui, |plot_ui| {
                    if !self.latency_history.is_empty() {
                        let points: PlotPoints = self
                            .latency_history
                            .iter()
                            .enumerate()
                            .map(|(i, &v)| [i as f64, v])
                            .collect();

                        plot_ui.line(
                            Line::new(points)
                                .color(Color32::from_rgb(255, 150, 100))
                                .width(2.0),
                        );
                    }
                });
            });
        });
    }

    /// Render export options
    fn render_export(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.label("Export:");

            if ui.button("Copy JSON").clicked() {
                if let Some(ref metrics) = self.latest_metrics {
                    let report = BenchmarkReport::new(
                        &self.selected_waveform,
                        self.sample_rate,
                        self.batch_size,
                        self.port,
                        self.format.name(),
                        metrics,
                    );
                    ui.output_mut(|o| o.copied_text = report.to_json());
                    self.status_message = "JSON copied to clipboard".to_string();
                }
            }

            if ui.button("Copy CSV").clicked() {
                if let Some(ref metrics) = self.latest_metrics {
                    let report = BenchmarkReport::new(
                        &self.selected_waveform,
                        self.sample_rate,
                        self.batch_size,
                        self.port,
                        self.format.name(),
                        metrics,
                    );
                    let csv = format!("{}\n{}", BenchmarkReport::csv_header(), report.to_csv_row());
                    ui.output_mut(|o| o.copied_text = csv);
                    self.status_message = "CSV copied to clipboard".to_string();
                }
            }

            if ui.button("Copy Text Report").clicked() {
                if let Some(ref metrics) = self.latest_metrics {
                    let report = BenchmarkReport::new(
                        &self.selected_waveform,
                        self.sample_rate,
                        self.batch_size,
                        self.port,
                        self.format.name(),
                        metrics,
                    );
                    ui.output_mut(|o| o.copied_text = report.to_text());
                    self.status_message = "Report copied to clipboard".to_string();
                }
            }
        });
    }

    /// Start the benchmark
    fn start_benchmark(&mut self) {
        // Reset state
        self.stop_signal.store(false, Ordering::SeqCst);
        self.throughput_history.clear();
        self.latency_history.clear();
        self.latest_metrics = None;
        self.last_report = None;

        // Create channel for metrics
        let (tx, rx) = std::sync::mpsc::channel();
        self.metrics_rx = Some(rx);

        // Clone values for thread
        let port = self.port;
        let format = self.format;
        let waveform = self.selected_waveform.clone();
        let sample_rate = self.sample_rate;
        let batch_size = self.batch_size;
        let stop_signal = self.stop_signal.clone();

        // Spawn benchmark thread
        std::thread::spawn(move || {
            Self::run_benchmark(port, format, waveform, sample_rate, batch_size, stop_signal, tx);
        });

        self.state = BenchmarkState::Running;
        self.status_message = format!("Listening on UDP port {}", self.port);
        self.last_update = Instant::now();
    }

    /// Stop the benchmark
    fn stop_benchmark(&mut self) {
        self.stop_signal.store(true, Ordering::SeqCst);
        self.state = BenchmarkState::Stopped;
        self.status_message = "Benchmark stopped".to_string();
    }

    /// Clear results
    fn clear_results(&mut self) {
        self.throughput_history.clear();
        self.latency_history.clear();
        self.latest_metrics = None;
        self.last_report = None;
        self.state = BenchmarkState::Idle;
        self.status_message = "Ready".to_string();
    }

    /// Poll for new metrics from background thread
    fn poll_metrics(&mut self) {
        if let Some(ref rx) = self.metrics_rx {
            // Drain all available metrics
            while let Ok(metrics) = rx.try_recv() {
                // Update history
                self.throughput_history.push(metrics.throughput_samples_per_sec);
                self.latency_history.push(metrics.avg_latency_us);

                // Limit history size
                if self.throughput_history.len() > self.max_history {
                    self.throughput_history.remove(0);
                }
                if self.latency_history.len() > self.max_history {
                    self.latency_history.remove(0);
                }

                self.latest_metrics = Some(metrics);
                self.last_update = Instant::now();
            }

            // Update status
            if self.state == BenchmarkState::Running {
                let elapsed = self.last_update.elapsed();
                if elapsed > Duration::from_secs(5) {
                    self.status_message = format!(
                        "Waiting for data on port {} ({:.0}s since last packet)",
                        self.port,
                        elapsed.as_secs_f64()
                    );
                } else if let Some(ref m) = self.latest_metrics {
                    self.status_message = format!(
                        "{:.0} Sps | {:.1} µs avg | {} symbols",
                        m.throughput_samples_per_sec, m.avg_latency_us, m.symbols_detected
                    );
                }
            }
        }
    }

    /// Background benchmark loop
    fn run_benchmark(
        port: u16,
        format: SampleFormat,
        waveform: String,
        sample_rate: f64,
        _batch_size: usize,
        stop_signal: Arc<AtomicBool>,
        tx: std::sync::mpsc::Sender<MetricsSummary>,
    ) {
        // Create receiver
        let mut receiver = match BenchmarkReceiver::bind(port, format) {
            Ok(r) => r,
            Err(e) => {
                tracing::error!("Failed to bind UDP port {}: {}", port, e);
                return;
            }
        };

        // Create waveform runner
        let runner = match WaveformRunner::new(&waveform, sample_rate) {
            Ok(r) => r,
            Err(e) => {
                tracing::error!("Failed to create waveform runner: {}", e);
                return;
            }
        };

        // Metrics collector
        let mut metrics = BenchmarkMetrics::new();
        let mut last_report = Instant::now();

        loop {
            if stop_signal.load(Ordering::SeqCst) {
                break;
            }

            // Receive samples with timeout
            match receiver.recv_batch(Duration::from_millis(100)) {
                Ok(samples) => {
                    if samples.is_empty() {
                        continue;
                    }

                    // Record receive stats
                    let bytes = samples.len() * 8; // f32 format: 8 bytes per sample
                    metrics.record_receive(samples.len(), bytes);

                    // Process through waveform
                    let result = runner.process(&samples);

                    // Update metrics with processing result
                    metrics.update(&result);

                    // Send summary every second
                    if last_report.elapsed() >= Duration::from_secs(1) {
                        let summary = metrics.summary();
                        let _ = tx.send(summary);
                        last_report = Instant::now();
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut
                    || e.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    // Timeout, check stop signal
                    continue;
                }
                Err(e) => {
                    tracing::warn!("UDP receive error: {}", e);
                    metrics.record_receive_error();
                }
            }
        }

        // Send final summary
        let _ = tx.send(metrics.summary());
    }
}
