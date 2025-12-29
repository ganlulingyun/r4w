//! Spectrum analyzer view

use egui::Ui;
use egui_plot::{Line, Plot, PlotPoints};
use r4w_core::fft_utils::FftProcessor;
use r4w_core::types::IQSample;

pub struct SpectrumView {
    fft_size: usize,
    window_pos: usize,
    show_magnitude: bool,
    show_phase: bool,
    log_scale: bool,
}

impl SpectrumView {
    pub fn new() -> Self {
        Self {
            fft_size: 256,
            window_pos: 0,
            show_magnitude: true,
            show_phase: false,
            log_scale: true,
        }
    }

    /// Find the largest power of 2 less than or equal to n
    fn largest_power_of_2_le(n: usize) -> usize {
        if n == 0 {
            return 0;
        }
        // Find highest set bit
        let mut power = 1;
        while power * 2 <= n {
            power *= 2;
        }
        power
    }

    pub fn render(&mut self, ui: &mut Ui, sample_rate: f64, waveform_name: &str, samples: &Option<Vec<IQSample>>) {
        ui.heading("Spectrum Analysis");
        ui.add_space(8.0);

        ui.label(format!(
            "The spectrum shows the frequency content of the {} signal. \
            The FFT converts time-domain samples to frequency-domain.",
            waveform_name
        ));

        ui.add_space(12.0);

        // Controls
        ui.horizontal(|ui| {
            ui.label("FFT Size:");
            egui::ComboBox::from_id_salt("fft_size")
                .selected_text(format!("{}", self.fft_size))
                .show_ui(ui, |ui| {
                    for size in [64, 128, 256, 512, 1024] {
                        ui.selectable_value(&mut self.fft_size, size, format!("{}", size));
                    }
                });

            ui.checkbox(&mut self.show_magnitude, "Magnitude");
            ui.checkbox(&mut self.show_phase, "Phase");
            ui.checkbox(&mut self.log_scale, "Log Scale (dB)");
        });

        if let Some(ref signal) = samples {
            let max_pos = signal.len().saturating_sub(self.fft_size);
            ui.add(
                egui::Slider::new(&mut self.window_pos, 0..=max_pos)
                    .text("Window Position")
                    .clamping(egui::SliderClamping::Always),
            );

            // Get window of samples
            let end = (self.window_pos + self.fft_size).min(signal.len());
            let window = &signal[self.window_pos..end];

            if window.len() >= self.fft_size {
                let mut fft = FftProcessor::new(self.fft_size);
                let spectrum = fft.fft(window);

                // Calculate frequency axis
                let freq_resolution = sample_rate / self.fft_size as f64;
                let frequencies: Vec<f64> = (0..self.fft_size)
                    .map(|i| {
                        if i < self.fft_size / 2 {
                            i as f64 * freq_resolution
                        } else {
                            (i as f64 - self.fft_size as f64) * freq_resolution
                        }
                    })
                    .collect();

                // FFT shift for display
                let shifted_freqs = FftProcessor::fft_shift(&frequencies);

                // Magnitude
                if self.show_magnitude {
                    ui.add_space(12.0);
                    ui.heading("Magnitude Spectrum");

                    let magnitudes = if self.log_scale {
                        FftProcessor::power_spectrum_db(&spectrum)
                    } else {
                        FftProcessor::magnitude_spectrum(&spectrum)
                    };
                    let shifted_mags = FftProcessor::fft_shift(&magnitudes);

                    let plot = Plot::new("magnitude_plot")
                        .height(250.0)
                        .allow_zoom(true)
                        .allow_drag(true)
                        .x_axis_label("Frequency (Hz)")
                        .y_axis_label(if self.log_scale { "Power (dB)" } else { "Magnitude" });

                    plot.show(ui, |plot_ui| {
                        let points: PlotPoints = shifted_freqs
                            .iter()
                            .zip(shifted_mags.iter())
                            .map(|(&f, &m)| [f, m])
                            .collect();

                        plot_ui.line(
                            Line::new(points)
                                .name("Magnitude")
                                .color(egui::Color32::GREEN),
                        );
                    });
                }

                // Phase
                if self.show_phase {
                    ui.add_space(12.0);
                    ui.heading("Phase Spectrum");

                    let phases: Vec<f64> = spectrum.iter().map(|c| c.arg().to_degrees()).collect();
                    let shifted_phases = FftProcessor::fft_shift(&phases);

                    let plot = Plot::new("phase_plot")
                        .height(200.0)
                        .allow_zoom(true)
                        .x_axis_label("Frequency (Hz)")
                        .y_axis_label("Phase (degrees)");

                    plot.show(ui, |plot_ui| {
                        let points: PlotPoints = shifted_freqs
                            .iter()
                            .zip(shifted_phases.iter())
                            .map(|(&f, &p)| [f, p])
                            .collect();

                        plot_ui.line(
                            Line::new(points)
                                .name("Phase")
                                .color(egui::Color32::YELLOW),
                        );
                    });
                }

                // Info
                ui.add_space(8.0);
                ui.label(format!(
                    "Window: {} samples | Freq resolution: {:.1} Hz | Sample rate: {:.0} kHz",
                    self.fft_size,
                    freq_resolution,
                    sample_rate / 1000.0
                ));
            } else {
                // Not enough samples for FFT
                ui.add_space(12.0);
                ui.colored_label(
                    egui::Color32::YELLOW,
                    format!(
                        "Not enough samples for spectrum analysis. \
                        Signal has {} samples, but FFT requires {} samples.",
                        signal.len(),
                        self.fft_size
                    ),
                );
                ui.add_space(8.0);

                // Calculate the best FFT size that fits
                let best_fft_size = Self::largest_power_of_2_le(signal.len());

                if best_fft_size >= 8 {
                    ui.horizontal(|ui| {
                        if ui.button(format!("Auto-fit FFT size to {}", best_fft_size)).clicked() {
                            self.fft_size = best_fft_size;
                        }
                        ui.label("(largest power of 2 that fits)");
                    });
                    ui.add_space(8.0);
                } else {
                    ui.label("Signal too short for any meaningful FFT analysis.");
                    ui.add_space(8.0);
                }

                ui.label("Or try one of the following:");
                ui.indent("fft_suggestions", |ui| {
                    ui.label("• Increase signal duration (add more test bits)");
                    ui.label("• Lower the symbol rate to get more samples per symbol");
                    ui.label("• Increase the sample rate");
                });
            }
        } else {
            ui.label("Generate a signal to analyze its spectrum.");
        }

        ui.add_space(20.0);
        ui.separator();

        ui.collapsing("Understanding the Spectrum", |ui| {
            ui.label("• The FFT converts time-domain samples to frequency-domain");
            ui.label("• Peak locations show the dominant frequencies in the signal");
            ui.label("• Bandwidth occupied shows how much spectrum the signal uses");
            ui.label("• Log scale (dB) makes it easier to see weak signals");
            ui.label("• Different modulation types have characteristic spectral shapes");
        });
    }
}
