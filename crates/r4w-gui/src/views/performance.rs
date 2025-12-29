//! Performance View - Compare sequential vs parallel implementations
//!
//! Shows side-by-side performance comparisons with speedup metrics,
//! system information, and hardware limitation analysis.
//!
//! Also compares SIMD (auto-vectorized) vs scalar implementations.

use egui::{Color32, RichText, Ui};
use egui_plot::{Bar, BarChart, Legend, Line, Plot, PlotPoints};
use rayon::prelude::*;
use r4w_core::fft_utils::Spectrogram;
use r4w_core::modulation::Modulator;
use r4w_core::parallel::{ParallelDemodulator, ParallelModulator};
use r4w_core::params::LoRaParams;
use r4w_core::simd_utils;
use r4w_core::types::{Complex, IQSample};
use std::time::Instant;

/// Comparison mode for benchmarks
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ComparisonMode {
    /// Compare sequential vs parallel (Rayon)
    #[default]
    SequentialVsParallel,
    /// Compare SIMD (auto-vectorized) vs scalar
    SimdVsScalar,
}

/// Benchmark operation types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BenchmarkOperation {
    BatchModulation,
    SymbolDemodulation,
    Spectrogram,
    Magnitude,
    Power,
    ComplexMultiply,
    Correlation,
    FrequencyShift,
    WindowFunction,
}

impl BenchmarkOperation {
    pub fn name(&self) -> &'static str {
        match self {
            Self::BatchModulation => "Batch Modulation",
            Self::SymbolDemodulation => "Symbol Demodulation",
            Self::Spectrogram => "Spectrogram (FFT)",
            Self::Magnitude => "Magnitude Computation",
            Self::Power => "Power Computation",
            Self::ComplexMultiply => "Complex Multiply",
            Self::Correlation => "Correlation",
            Self::FrequencyShift => "Frequency Shift",
            Self::WindowFunction => "Window Function",
        }
    }

    pub fn description(&self) -> &'static str {
        match self {
            Self::BatchModulation => "Modulate multiple payloads using LoRa CSS",
            Self::SymbolDemodulation => "Demodulate symbols from I/Q samples",
            Self::Spectrogram => "Compute FFT frames for spectrogram",
            Self::Magnitude => "Compute |z| for complex samples",
            Self::Power => "Compute |z|^2 for complex samples",
            Self::ComplexMultiply => "Element-wise complex multiplication",
            Self::Correlation => "Sliding window correlation",
            Self::FrequencyShift => "Apply frequency offset to signal",
            Self::WindowFunction => "Generate Hann window coefficients",
        }
    }

    /// Operations available for Sequential vs Parallel comparison
    pub fn parallel_ops() -> &'static [BenchmarkOperation] {
        &[
            Self::BatchModulation,
            Self::SymbolDemodulation,
            Self::Spectrogram,
            Self::Magnitude,
            Self::Power,
            Self::ComplexMultiply,
            Self::Correlation,
        ]
    }

    /// Operations available for SIMD vs Scalar comparison
    pub fn simd_ops() -> &'static [BenchmarkOperation] {
        &[
            Self::Magnitude,
            Self::Power,
            Self::ComplexMultiply,
            Self::Correlation,
            Self::FrequencyShift,
            Self::WindowFunction,
        ]
    }

    pub fn sizes(&self) -> &'static [usize] {
        match self {
            Self::BatchModulation => &[1, 4, 16, 64],
            Self::SymbolDemodulation => &[4, 16, 64, 256],
            Self::Spectrogram => &[4096, 16384, 65536],
            Self::Magnitude | Self::Power | Self::ComplexMultiply => &[1000, 10000, 100000],
            Self::Correlation => &[256, 1024, 4096],
            Self::FrequencyShift => &[1000, 10000, 100000],
            Self::WindowFunction => &[256, 1024, 4096],
        }
    }

    pub fn size_unit(&self) -> &'static str {
        match self {
            Self::BatchModulation => "messages",
            Self::SymbolDemodulation => "symbols",
            Self::Spectrogram => "samples",
            Self::Magnitude | Self::Power | Self::ComplexMultiply => "samples",
            Self::Correlation => "samples",
            Self::FrequencyShift => "samples",
            Self::WindowFunction => "samples",
        }
    }
}

/// Results from a benchmark run
#[derive(Debug, Clone)]
pub struct BenchmarkResults {
    pub operation: BenchmarkOperation,
    pub size: usize,
    pub mode: ComparisonMode,
    /// Time for first variant (sequential or scalar)
    pub first_time_us: f64,
    /// Time for second variant (parallel or SIMD)
    pub second_time_us: f64,
    pub speedup: f64,
    pub efficiency: f64,
    pub throughput_first: f64,
    pub throughput_second: f64,
}

impl BenchmarkResults {
    /// Get label for first variant
    pub fn first_label(&self) -> &'static str {
        match self.mode {
            ComparisonMode::SequentialVsParallel => "Sequential",
            ComparisonMode::SimdVsScalar => "Scalar",
        }
    }

    /// Get label for second variant
    pub fn second_label(&self) -> &'static str {
        match self.mode {
            ComparisonMode::SequentialVsParallel => "Parallel",
            ComparisonMode::SimdVsScalar => "SIMD",
        }
    }
}

/// Scaling data point for charting
#[derive(Debug, Clone)]
pub struct ScalingDataPoint {
    pub size: usize,
    pub sequential_time: f64,
    pub parallel_time: f64,
    pub speedup: f64,
}

/// System information
#[derive(Debug, Clone)]
pub struct SystemInfo {
    pub logical_cores: usize,
    pub rayon_threads: usize,
    pub arch: String,
}

impl SystemInfo {
    pub fn collect() -> Self {
        Self {
            logical_cores: std::thread::available_parallelism()
                .map(|p| p.get())
                .unwrap_or(1),
            rayon_threads: rayon::current_num_threads(),
            arch: std::env::consts::ARCH.to_string(),
        }
    }
}

/// Performance comparison view
pub struct PerformanceView {
    /// Current comparison mode
    comparison_mode: ComparisonMode,

    /// Selected benchmark operation
    selected_operation: BenchmarkOperation,

    /// Selected size index
    selected_size_idx: usize,

    /// Auto-run scaling analysis
    #[allow(dead_code)]
    auto_run_scaling: bool,

    /// Latest benchmark results
    results: Option<BenchmarkResults>,

    /// Scaling analysis data
    scaling_data: Vec<ScalingDataPoint>,

    /// Is a benchmark currently running
    #[allow(dead_code)]
    is_running: bool,

    /// System information (cached)
    system_info: SystemInfo,

    /// Number of iterations for timing stability
    iterations: usize,
}

impl Default for PerformanceView {
    fn default() -> Self {
        Self::new()
    }
}

impl PerformanceView {
    pub fn new() -> Self {
        Self {
            comparison_mode: ComparisonMode::SequentialVsParallel,
            selected_operation: BenchmarkOperation::BatchModulation,
            selected_size_idx: 1,
            auto_run_scaling: false,
            results: None,
            scaling_data: Vec::new(),
            is_running: false,
            system_info: SystemInfo::collect(),
            iterations: 5,
        }
    }

    pub fn render(&mut self, ui: &mut Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.heading("Performance Optimizations");
            ui.add_space(8.0);

            let description = match self.comparison_mode {
                ComparisonMode::SequentialVsParallel => {
                    "Compare sequential vs parallel (Rayon) implementations to understand \
                     when multi-threading is beneficial."
                }
                ComparisonMode::SimdVsScalar => {
                    "Compare SIMD (auto-vectorized) vs scalar implementations to see \
                     the benefits of CPU vector instructions (SSE/AVX/NEON)."
                }
            };
            ui.label(description);
            ui.add_space(16.0);

            // System Information
            self.render_system_info(ui);

            ui.add_space(16.0);
            ui.separator();

            // Benchmark Controls
            self.render_controls(ui);

            ui.add_space(16.0);
            ui.separator();

            // Results
            if let Some(ref results) = self.results {
                self.render_results(ui, results);
                ui.add_space(16.0);
            }

            // Scaling Analysis
            if !self.scaling_data.is_empty() {
                self.render_scaling_chart(ui);
                ui.add_space(16.0);
            }

            // Recommendations
            self.render_recommendations(ui);
        });
    }

    fn render_system_info(&self, ui: &mut Ui) {
        ui.group(|ui| {
            ui.heading("System Information");
            ui.add_space(8.0);

            egui::Grid::new("system_info_grid")
                .num_columns(2)
                .spacing([20.0, 4.0])
                .show(ui, |ui| {
                    ui.label("CPU Cores:");
                    ui.label(
                        RichText::new(format!("{} logical", self.system_info.logical_cores))
                            .strong(),
                    );
                    ui.end_row();

                    ui.label("Rayon Threads:");
                    ui.label(
                        RichText::new(format!("{}", self.system_info.rayon_threads)).strong(),
                    );
                    ui.end_row();

                    ui.label("Architecture:");
                    ui.label(RichText::new(&self.system_info.arch).strong());
                    ui.end_row();

                    ui.label("SIMD Support:");
                    let simd_info = if cfg!(target_arch = "x86_64") {
                        "SSE4.2, AVX2 (if available)"
                    } else if cfg!(target_arch = "aarch64") {
                        "NEON"
                    } else {
                        "Auto-vectorization"
                    };
                    ui.label(RichText::new(simd_info).strong());
                    ui.end_row();
                });
        });
    }

    fn render_controls(&mut self, ui: &mut Ui) {
        ui.group(|ui| {
            ui.heading("Benchmark Controls");
            ui.add_space(8.0);

            // Mode selector
            ui.horizontal(|ui| {
                ui.label("Comparison Mode:");
                let old_mode = self.comparison_mode;
                egui::ComboBox::from_id_salt("mode_select")
                    .selected_text(match self.comparison_mode {
                        ComparisonMode::SequentialVsParallel => "Sequential vs Parallel",
                        ComparisonMode::SimdVsScalar => "SIMD vs Scalar",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut self.comparison_mode,
                            ComparisonMode::SequentialVsParallel,
                            "Sequential vs Parallel (Multi-threading)",
                        );
                        ui.selectable_value(
                            &mut self.comparison_mode,
                            ComparisonMode::SimdVsScalar,
                            "SIMD vs Scalar (Vectorization)",
                        );
                    });

                // Reset operation selection if mode changed
                if old_mode != self.comparison_mode {
                    self.results = None;
                    self.scaling_data.clear();
                    // Select first available operation for new mode
                    let ops = match self.comparison_mode {
                        ComparisonMode::SequentialVsParallel => BenchmarkOperation::parallel_ops(),
                        ComparisonMode::SimdVsScalar => BenchmarkOperation::simd_ops(),
                    };
                    if !ops.is_empty() {
                        self.selected_operation = ops[0];
                        self.selected_size_idx = 0;
                    }
                }
            });

            ui.add_space(8.0);

            // Get available operations for current mode
            let available_ops = match self.comparison_mode {
                ComparisonMode::SequentialVsParallel => BenchmarkOperation::parallel_ops(),
                ComparisonMode::SimdVsScalar => BenchmarkOperation::simd_ops(),
            };

            ui.horizontal(|ui| {
                ui.label("Operation:");
                egui::ComboBox::from_id_salt("operation_select")
                    .selected_text(self.selected_operation.name())
                    .show_ui(ui, |ui| {
                        for op in available_ops {
                            ui.selectable_value(&mut self.selected_operation, *op, op.name());
                        }
                    });

                ui.add_space(16.0);

                let sizes = self.selected_operation.sizes();
                let unit = self.selected_operation.size_unit();
                ui.label("Size:");
                egui::ComboBox::from_id_salt("size_select")
                    .selected_text(format!(
                        "{} {}",
                        sizes.get(self.selected_size_idx).copied().unwrap_or(sizes[0]),
                        unit
                    ))
                    .show_ui(ui, |ui| {
                        for (idx, &size) in sizes.iter().enumerate() {
                            ui.selectable_value(
                                &mut self.selected_size_idx,
                                idx,
                                format!("{} {}", size, unit),
                            );
                        }
                    });
            });

            // Clamp size index if operation changed
            let max_idx = self.selected_operation.sizes().len().saturating_sub(1);
            if self.selected_size_idx > max_idx {
                self.selected_size_idx = max_idx;
            }

            ui.add_space(8.0);
            ui.label(
                RichText::new(self.selected_operation.description())
                    .italics()
                    .weak(),
            );

            ui.add_space(12.0);
            ui.horizontal(|ui| {
                if ui
                    .button(RichText::new("Run Benchmark").strong())
                    .clicked()
                {
                    self.run_single_benchmark();
                }

                ui.add_space(16.0);

                if ui.button("Run Scaling Analysis").clicked() {
                    self.run_scaling_analysis();
                }

                ui.add_space(16.0);

                ui.label("Iterations:");
                ui.add(egui::DragValue::new(&mut self.iterations).range(1..=20));
            });
        });
    }

    fn render_results(&self, ui: &mut Ui, results: &BenchmarkResults) {
        ui.group(|ui| {
            ui.heading(format!(
                "Results: {} ({} {})",
                results.operation.name(),
                results.size,
                results.operation.size_unit()
            ));
            ui.add_space(8.0);

            // Bar chart comparison
            let max_time = results.first_time_us.max(results.second_time_us);
            let chart_height = 120.0;

            // Colors: blue for first (slower), green for second (faster)
            let first_color = match results.mode {
                ComparisonMode::SequentialVsParallel => Color32::from_rgb(66, 133, 244), // Blue
                ComparisonMode::SimdVsScalar => Color32::from_rgb(234, 67, 53), // Red for scalar
            };
            let second_color = Color32::from_rgb(52, 168, 83); // Green for faster

            Plot::new("comparison_chart")
                .height(chart_height)
                .allow_zoom(false)
                .allow_drag(false)
                .allow_scroll(false)
                .allow_boxed_zoom(false)
                .show_x(false)
                .show_y(true)
                .y_axis_label("Time (us)")
                .include_y(0.0)
                .include_y(max_time * 1.1)
                .show(ui, |plot_ui| {
                    // First variant bar
                    plot_ui.bar_chart(
                        BarChart::new(vec![Bar::new(0.5, results.first_time_us)
                            .name(results.first_label())
                            .width(0.35)])
                        .color(first_color)
                        .name(results.first_label()),
                    );

                    // Second variant bar
                    plot_ui.bar_chart(
                        BarChart::new(vec![Bar::new(1.5, results.second_time_us)
                            .name(results.second_label())
                            .width(0.35)])
                        .color(second_color)
                        .name(results.second_label()),
                    );
                });

            ui.add_space(8.0);

            // Metrics
            egui::Grid::new("results_metrics")
                .num_columns(4)
                .spacing([30.0, 4.0])
                .show(ui, |ui| {
                    ui.label(format!("{}:", results.first_label()));
                    ui.label(RichText::new(format!("{:.1} us", results.first_time_us)).strong());
                    ui.label("Throughput:");
                    ui.label(
                        RichText::new(format!("{:.1}K/s", results.throughput_first / 1000.0)).strong(),
                    );
                    ui.end_row();

                    ui.label(format!("{}:", results.second_label()));
                    ui.label(RichText::new(format!("{:.1} us", results.second_time_us)).strong());
                    ui.label("Throughput:");
                    ui.label(
                        RichText::new(format!("{:.1}K/s", results.throughput_second / 1000.0)).strong(),
                    );
                    ui.end_row();
                });

            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let speedup_color = if results.speedup >= 1.0 {
                    Color32::from_rgb(52, 168, 83) // Green
                } else {
                    Color32::from_rgb(234, 67, 53) // Red
                };

                ui.label("Speedup:");
                ui.label(
                    RichText::new(format!("{:.2}x", results.speedup))
                        .strong()
                        .color(speedup_color),
                );

                ui.add_space(30.0);

                let efficiency_label = match results.mode {
                    ComparisonMode::SequentialVsParallel => "Parallel Efficiency:",
                    ComparisonMode::SimdVsScalar => "Vectorization Benefit:",
                };
                ui.label(efficiency_label);
                ui.label(
                    RichText::new(format!("{:.1}%", results.efficiency * 100.0))
                        .strong()
                        .color(if results.efficiency > 0.5 {
                            Color32::from_rgb(52, 168, 83)
                        } else {
                            Color32::from_rgb(251, 188, 4) // Yellow
                        }),
                );
            });
        });
    }

    fn render_scaling_chart(&self, ui: &mut Ui) {
        ui.group(|ui| {
            ui.heading("Scaling Analysis");
            ui.add_space(8.0);

            let chart_height = 200.0;

            let (first_label, second_label, first_color) = match self.comparison_mode {
                ComparisonMode::SequentialVsParallel => (
                    "Sequential",
                    "Parallel",
                    Color32::from_rgb(66, 133, 244), // Blue
                ),
                ComparisonMode::SimdVsScalar => (
                    "Scalar",
                    "SIMD",
                    Color32::from_rgb(234, 67, 53), // Red
                ),
            };

            Plot::new("scaling_chart")
                .height(chart_height)
                .legend(Legend::default())
                .x_axis_label("Problem Size")
                .y_axis_label("Time (us)")
                .allow_zoom(true)
                .allow_drag(true)
                .show(ui, |plot_ui| {
                    // First variant line
                    let first_points: PlotPoints = self
                        .scaling_data
                        .iter()
                        .map(|p| [p.size as f64, p.sequential_time])
                        .collect();
                    plot_ui.line(
                        Line::new(first_points)
                            .name(first_label)
                            .color(first_color)
                            .width(2.0),
                    );

                    // Second variant line
                    let second_points: PlotPoints = self
                        .scaling_data
                        .iter()
                        .map(|p| [p.size as f64, p.parallel_time])
                        .collect();
                    plot_ui.line(
                        Line::new(second_points)
                            .name(second_label)
                            .color(Color32::from_rgb(52, 168, 83))
                            .width(2.0),
                    );
                });

            // Speedup summary
            ui.add_space(8.0);
            ui.horizontal_wrapped(|ui| {
                for point in &self.scaling_data {
                    let color = if point.speedup >= 1.0 {
                        Color32::from_rgb(52, 168, 83)
                    } else {
                        Color32::from_rgb(234, 67, 53)
                    };
                    ui.label(format!("Size {}: ", point.size));
                    ui.label(RichText::new(format!("{:.2}x", point.speedup)).color(color));
                    ui.add_space(16.0);
                }
            });
        });
    }

    fn render_recommendations(&self, ui: &mut Ui) {
        ui.group(|ui| {
            ui.heading("Recommendations");
            ui.add_space(8.0);

            match self.comparison_mode {
                ComparisonMode::SequentialVsParallel => {
                    // Find crossover point if we have scaling data
                    let crossover = self
                        .scaling_data
                        .iter()
                        .find(|p| p.speedup >= 1.0)
                        .map(|p| p.size);

                    if let Some(size) = crossover {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("OK").color(Color32::from_rgb(52, 168, 83)));
                            ui.label(format!(
                                "Use parallel for {} sizes >= {}",
                                self.selected_operation.size_unit(),
                                size
                            ));
                        });
                    }

                    // Check for overhead warning
                    if let Some(first) = self.scaling_data.first() {
                        if first.speedup < 1.0 {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("!!").color(Color32::from_rgb(234, 67, 53)));
                                ui.label("Sequential is faster for small workloads (thread overhead)");
                            });
                        }
                    }

                    // General recommendations
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        ui.label(RichText::new("i").color(Color32::from_rgb(66, 133, 244)));
                        ui.label(format!(
                            "Max theoretical speedup: {}x ({} cores)",
                            self.system_info.logical_cores, self.system_info.logical_cores
                        ));
                    });
                }
                ComparisonMode::SimdVsScalar => {
                    // SIMD-specific recommendations
                    if let Some(ref results) = self.results {
                        if results.speedup >= 2.0 {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("OK").color(Color32::from_rgb(52, 168, 83)));
                                ui.label(format!(
                                    "SIMD provides {:.1}x speedup - vectorization is effective!",
                                    results.speedup
                                ));
                            });
                        } else if results.speedup >= 1.0 {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("i").color(Color32::from_rgb(251, 188, 4)));
                                ui.label("Moderate SIMD benefit - may be memory-bound");
                            });
                        } else {
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("!!").color(Color32::from_rgb(234, 67, 53)));
                                ui.label("SIMD not providing benefit - check data dependencies");
                            });
                        }
                    }

                    ui.add_space(8.0);
                    let simd_info = if cfg!(target_arch = "x86_64") {
                        "x86-64: Can use SSE (128-bit, 2 f64), AVX (256-bit, 4 f64), AVX-512 (8 f64)"
                    } else if cfg!(target_arch = "aarch64") {
                        "ARM64: Using NEON (128-bit, 2 f64 per instruction)"
                    } else {
                        "SIMD support depends on CPU architecture"
                    };
                    ui.horizontal(|ui| {
                        ui.label(RichText::new("i").color(Color32::from_rgb(66, 133, 244)));
                        ui.label(simd_info);
                    });

                    ui.horizontal(|ui| {
                        ui.label(RichText::new("i").color(Color32::from_rgb(66, 133, 244)));
                        ui.label("Theoretical max speedup: 2-8x depending on vector width");
                    });
                }
            }

            ui.horizontal(|ui| {
                ui.label(RichText::new("i").color(Color32::from_rgb(66, 133, 244)));
                ui.label("Compile with RUSTFLAGS=\"-C target-cpu=native\" for best SIMD");
            });

            if let Some(ref results) = self.results {
                let mem_estimate = match results.operation {
                    BenchmarkOperation::BatchModulation => {
                        // Each LoRa symbol ~ 128 samples * 16 bytes = 2KB
                        results.size * 10 * 2048 // ~20KB per message
                    }
                    BenchmarkOperation::Spectrogram => {
                        results.size * 16 // 16 bytes per complex sample
                    }
                    _ => results.size * 16,
                };

                ui.horizontal(|ui| {
                    ui.label(RichText::new("i").color(Color32::from_rgb(66, 133, 244)));
                    ui.label(format!(
                        "Estimated memory: ~{:.1} KB",
                        mem_estimate as f64 / 1024.0
                    ));
                });
            }
        });
    }

    fn run_single_benchmark(&mut self) {
        let sizes = self.selected_operation.sizes();
        let size = sizes.get(self.selected_size_idx).copied().unwrap_or(sizes[0]);
        self.results = Some(self.run_benchmark(self.selected_operation, size));
    }

    fn run_scaling_analysis(&mut self) {
        self.scaling_data.clear();

        for &size in self.selected_operation.sizes() {
            let results = self.run_benchmark(self.selected_operation, size);
            self.scaling_data.push(ScalingDataPoint {
                size,
                sequential_time: results.first_time_us,
                parallel_time: results.second_time_us,
                speedup: results.speedup,
            });
        }

        // Also update main results with last run
        if let Some(last) = self.scaling_data.last() {
            self.results = Some(self.run_benchmark(self.selected_operation, last.size));
        }
    }

    fn run_benchmark(&self, op: BenchmarkOperation, size: usize) -> BenchmarkResults {
        match self.comparison_mode {
            ComparisonMode::SequentialVsParallel => match op {
                BenchmarkOperation::BatchModulation => self.bench_batch_modulation(size),
                BenchmarkOperation::SymbolDemodulation => self.bench_symbol_demod(size),
                BenchmarkOperation::Spectrogram => self.bench_spectrogram(size),
                BenchmarkOperation::Magnitude => self.bench_magnitude_parallel(size),
                BenchmarkOperation::Power => self.bench_power_parallel(size),
                BenchmarkOperation::ComplexMultiply => self.bench_complex_multiply_parallel(size),
                BenchmarkOperation::Correlation => self.bench_correlation_parallel(size),
                BenchmarkOperation::FrequencyShift => self.bench_frequency_shift_simd(size),
                BenchmarkOperation::WindowFunction => self.bench_window_simd(size),
            },
            ComparisonMode::SimdVsScalar => match op {
                BenchmarkOperation::Magnitude => self.bench_magnitude_simd(size),
                BenchmarkOperation::Power => self.bench_power_simd(size),
                BenchmarkOperation::ComplexMultiply => self.bench_complex_multiply_simd(size),
                BenchmarkOperation::Correlation => self.bench_correlation_simd(size),
                BenchmarkOperation::FrequencyShift => self.bench_frequency_shift_simd(size),
                BenchmarkOperation::WindowFunction => self.bench_window_simd(size),
                _ => self.make_results(op, size, self.comparison_mode, 0.0, 0.0),
            },
        }
    }

    fn bench_batch_modulation(&self, batch_size: usize) -> BenchmarkResults {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .build();
        let payloads: Vec<Vec<u8>> = (0..batch_size)
            .map(|i| format!("Message {}", i).into_bytes())
            .collect();
        let payload_refs: Vec<&[u8]> = payloads.iter().map(|v| v.as_slice()).collect();

        // Sequential timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            for payload in &payload_refs {
                let mut modulator = Modulator::new(params.clone());
                let _ = modulator.modulate(payload);
            }
        }
        let sequential_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // Parallel timing
        let parallel_mod = ParallelModulator::new(params.clone());
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = parallel_mod.modulate_batch(&payload_refs);
        }
        let parallel_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::BatchModulation,
            batch_size,
            ComparisonMode::SequentialVsParallel,
            sequential_time,
            parallel_time,
        )
    }

    fn bench_symbol_demod(&self, num_symbols: usize) -> BenchmarkResults {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .build();

        // Generate test samples
        let mut modulator = Modulator::new(params.clone());
        let test_payload: Vec<u8> = (0..num_symbols as u8).collect();
        let samples = modulator.symbols_only(&test_payload);

        // Sequential timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let mut demodulator = r4w_core::demodulation::Demodulator::new(params.clone());
            let _ = demodulator.demodulate_symbols(&samples, num_symbols);
        }
        let sequential_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // Parallel timing
        let parallel_demod = ParallelDemodulator::new(params.clone());
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = parallel_demod.demodulate_symbols_parallel(&samples);
        }
        let parallel_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::SymbolDemodulation,
            num_symbols,
            ComparisonMode::SequentialVsParallel,
            sequential_time,
            parallel_time,
        )
    }

    fn bench_spectrogram(&self, num_samples: usize) -> BenchmarkResults {
        // Generate test signal
        let signal: Vec<IQSample> = (0..num_samples)
            .map(|i| {
                let t = i as f64 / 1000.0;
                let phase = 2.0 * std::f64::consts::PI * 100.0 * t;
                Complex::new(phase.cos(), phase.sin())
            })
            .collect();

        let fft_size = 256;
        let hop_size = 128;

        // Sequential timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = Spectrogram::compute(&signal, fft_size, hop_size, 1000.0);
        }
        let sequential_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // Parallel timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = r4w_core::parallel::parallel_spectrogram(&signal, fft_size, hop_size, 1000.0);
        }
        let parallel_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::Spectrogram,
            num_samples,
            ComparisonMode::SequentialVsParallel,
            sequential_time,
            parallel_time,
        )
    }

    // ========== Sequential vs Parallel benchmarks ==========

    fn bench_magnitude_parallel(&self, num_samples: usize) -> BenchmarkResults {
        let samples: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.001, i as f64 * 0.002))
            .collect();

        // Sequential timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::compute_magnitudes(&samples);
        }
        let sequential_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // Parallel timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _: Vec<f64> = samples.par_iter().map(|s| s.norm()).collect();
        }
        let parallel_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::Magnitude,
            num_samples,
            ComparisonMode::SequentialVsParallel,
            sequential_time,
            parallel_time,
        )
    }

    fn bench_power_parallel(&self, num_samples: usize) -> BenchmarkResults {
        let samples: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.001, i as f64 * 0.002))
            .collect();

        // Sequential timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::compute_power(&samples);
        }
        let sequential_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // Parallel timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _: Vec<f64> = samples.par_iter().map(|s| s.norm_sqr()).collect();
        }
        let parallel_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::Power,
            num_samples,
            ComparisonMode::SequentialVsParallel,
            sequential_time,
            parallel_time,
        )
    }

    fn bench_complex_multiply_parallel(&self, num_samples: usize) -> BenchmarkResults {
        let a: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.001, i as f64 * 0.002))
            .collect();
        let b: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.003, i as f64 * 0.004))
            .collect();

        // Sequential timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::complex_multiply(&a, &b);
        }
        let sequential_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // Parallel timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _: Vec<IQSample> = a.par_iter().zip(b.par_iter()).map(|(&x, &y)| x * y).collect();
        }
        let parallel_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::ComplexMultiply,
            num_samples,
            ComparisonMode::SequentialVsParallel,
            sequential_time,
            parallel_time,
        )
    }

    fn bench_correlation_parallel(&self, num_samples: usize) -> BenchmarkResults {
        let signal: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.001, i as f64 * 0.002))
            .collect();
        let reference: Vec<IQSample> = (0..64)
            .map(|i| Complex::new(i as f64 * 0.01, 0.0))
            .collect();

        // Sequential timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::sliding_correlation_magnitude(&signal, &reference);
        }
        let sequential_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // Parallel timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = r4w_core::parallel::utils::parallel_correlate(&signal, &reference);
        }
        let parallel_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::Correlation,
            num_samples,
            ComparisonMode::SequentialVsParallel,
            sequential_time,
            parallel_time,
        )
    }

    // ========== SIMD vs Scalar benchmarks ==========

    fn bench_magnitude_simd(&self, num_samples: usize) -> BenchmarkResults {
        let samples: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.001, i as f64 * 0.002))
            .collect();

        // Scalar timing (non-vectorized)
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::scalar_compute_magnitudes(&samples);
        }
        let scalar_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // SIMD timing (auto-vectorized)
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::compute_magnitudes(&samples);
        }
        let simd_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::Magnitude,
            num_samples,
            ComparisonMode::SimdVsScalar,
            scalar_time,
            simd_time,
        )
    }

    fn bench_power_simd(&self, num_samples: usize) -> BenchmarkResults {
        let samples: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.001, i as f64 * 0.002))
            .collect();

        // Scalar timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::scalar_compute_power(&samples);
        }
        let scalar_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // SIMD timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::compute_power(&samples);
        }
        let simd_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::Power,
            num_samples,
            ComparisonMode::SimdVsScalar,
            scalar_time,
            simd_time,
        )
    }

    fn bench_complex_multiply_simd(&self, num_samples: usize) -> BenchmarkResults {
        let a: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.001, i as f64 * 0.002))
            .collect();
        let b: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.003, i as f64 * 0.004))
            .collect();

        // Scalar timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::scalar_complex_multiply(&a, &b);
        }
        let scalar_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // SIMD timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::complex_multiply(&a, &b);
        }
        let simd_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::ComplexMultiply,
            num_samples,
            ComparisonMode::SimdVsScalar,
            scalar_time,
            simd_time,
        )
    }

    fn bench_correlation_simd(&self, num_samples: usize) -> BenchmarkResults {
        let signal: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.001, i as f64 * 0.002))
            .collect();
        let reference: Vec<IQSample> = (0..64)
            .map(|i| Complex::new(i as f64 * 0.01, 0.0))
            .collect();

        // Scalar timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::scalar_correlate(&signal, &reference);
        }
        let scalar_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // SIMD timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::sliding_correlation_magnitude(&signal, &reference);
        }
        let simd_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::Correlation,
            num_samples,
            ComparisonMode::SimdVsScalar,
            scalar_time,
            simd_time,
        )
    }

    fn bench_frequency_shift_simd(&self, num_samples: usize) -> BenchmarkResults {
        let samples: Vec<IQSample> = (0..num_samples)
            .map(|i| Complex::new(i as f64 * 0.001, i as f64 * 0.002))
            .collect();
        let freq_shift = 1000.0;
        let sample_rate = 125000.0;

        // Scalar timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::scalar_frequency_shift(&samples, freq_shift, sample_rate);
        }
        let scalar_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // SIMD timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::frequency_shift(&samples, freq_shift, sample_rate);
        }
        let simd_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::FrequencyShift,
            num_samples,
            ComparisonMode::SimdVsScalar,
            scalar_time,
            simd_time,
        )
    }

    fn bench_window_simd(&self, size: usize) -> BenchmarkResults {
        // Scalar timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::scalar_hann_window(size);
        }
        let scalar_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        // SIMD timing
        let start = Instant::now();
        for _ in 0..self.iterations {
            let _ = simd_utils::hann_window(size);
        }
        let simd_time = start.elapsed().as_micros() as f64 / self.iterations as f64;

        self.make_results(
            BenchmarkOperation::WindowFunction,
            size,
            ComparisonMode::SimdVsScalar,
            scalar_time,
            simd_time,
        )
    }

    fn make_results(
        &self,
        operation: BenchmarkOperation,
        size: usize,
        mode: ComparisonMode,
        first_time: f64,
        second_time: f64,
    ) -> BenchmarkResults {
        let speedup = if second_time > 0.0 {
            first_time / second_time
        } else {
            1.0
        };

        // Efficiency calculation depends on mode
        let efficiency = match mode {
            ComparisonMode::SequentialVsParallel => speedup / self.system_info.logical_cores as f64,
            ComparisonMode::SimdVsScalar => {
                // For SIMD, theoretical max is ~2-4x on 128-bit vectors with f64
                speedup / 4.0
            }
        };

        // Throughput in elements/second
        let throughput_first = if first_time > 0.0 {
            size as f64 / (first_time / 1_000_000.0)
        } else {
            0.0
        };

        let throughput_second = if second_time > 0.0 {
            size as f64 / (second_time / 1_000_000.0)
        } else {
            0.0
        };

        BenchmarkResults {
            operation,
            size,
            mode,
            first_time_us: first_time,
            second_time_us: second_time,
            speedup,
            efficiency,
            throughput_first,
            throughput_second,
        }
    }
}
