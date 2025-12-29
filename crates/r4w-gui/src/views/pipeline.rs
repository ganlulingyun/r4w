//! Full TX/RX pipeline visualization

use egui::Ui;
use r4w_core::demodulation::Demodulator;
use r4w_core::modulation::Modulator;
use r4w_core::params::LoRaParams;
use r4w_sim::Channel;

pub struct PipelineView {
    run_full_pipeline: bool,
    tx_payload: Vec<u8>,
    rx_payload: Option<Vec<u8>>,
    symbols_match: Option<(usize, usize)>,
}

impl PipelineView {
    pub fn new() -> Self {
        Self {
            run_full_pipeline: false,
            tx_payload: Vec::new(),
            rx_payload: None,
            symbols_match: None,
        }
    }

    pub fn render(
        &mut self,
        ui: &mut Ui,
        params: &LoRaParams,
        payload: &str,
        modulator: &mut Option<Modulator>,
        channel: &mut Channel,
        demodulator: &mut Option<Demodulator>,
    ) {
        ui.heading("Complete TX → Channel → RX Pipeline");
        ui.add_space(8.0);

        ui.label(
            "This view shows the complete signal path from transmitter through \
            the wireless channel to the receiver.",
        );

        ui.add_space(12.0);

        // Pipeline diagram
        ui.code(
            r#"
┌──────────────┐    ┌─────────────┐    ┌──────────────┐
│  TRANSMITTER │───►│   CHANNEL   │───►│   RECEIVER   │
│              │    │             │    │              │
│ • Whitening  │    │ • AWGN      │    │ • Sync       │
│ • FEC        │    │ • Fading    │    │ • Demod      │
│ • Interleave │    │ • CFO       │    │ • FEC decode │
│ • Gray code  │    │ • Multipath │    │ • De-whiten  │
│ • CSS mod    │    │             │    │              │
└──────────────┘    └─────────────┘    └──────────────┘
"#,
        );

        ui.add_space(12.0);
        ui.separator();

        // Run pipeline
        if ui.button("Run Full Pipeline").clicked() {
            self.run_full_pipeline = true;
        }

        if self.run_full_pipeline {
            self.run_full_pipeline = false;

            // TX
            if let Some(ref mut mod_ref) = modulator {
                let tx_bytes = payload.as_bytes();
                self.tx_payload = tx_bytes.to_vec();

                mod_ref.enable_stage_recording();
                let tx_samples = mod_ref.modulate(tx_bytes);

                // Channel
                let rx_samples = channel.apply(&tx_samples);

                // RX
                if let Some(ref mut demod_ref) = demodulator {
                    demod_ref.enable_stage_recording();

                    // Skip preamble (simplified)
                    let n = params.samples_per_symbol();
                    let preamble_len = (params.preamble_length + 4) * n + n / 4;

                    if rx_samples.len() > preamble_len {
                        let payload_samples = &rx_samples[preamble_len..];

                        match demod_ref.demodulate(payload_samples) {
                            Ok(result) => {
                                self.rx_payload = Some(result.payload);

                                // Compare symbols
                                let tx_symbols = mod_ref.get_symbols(tx_bytes);
                                let matching = tx_symbols
                                    .iter()
                                    .zip(result.symbols.iter())
                                    .filter(|(a, b)| a == b)
                                    .count();
                                self.symbols_match = Some((matching, tx_symbols.len()));
                            }
                            Err(e) => {
                                ui.label(format!("Demodulation error: {}", e));
                            }
                        }
                    }
                }
            }
        }

        // Results
        ui.add_space(12.0);

        ui.columns(3, |columns| {
            // TX column
            columns[0].group(|ui| {
                ui.heading("Transmitted");
                ui.add_space(4.0);

                ui.label("Payload:");
                ui.code(payload);

                ui.add_space(4.0);
                ui.label("Bytes:");
                let hex: String = self.tx_payload.iter().map(|b| format!("{:02X} ", b)).collect();
                ui.code(&hex);
            });

            // Channel column
            columns[1].group(|ui| {
                ui.heading("Channel");
                ui.add_space(4.0);

                let ch_config = channel.config();
                ui.label(format!("Model: {:?}", ch_config.model));
                ui.label(format!("SNR: {:.1} dB", ch_config.snr_db));
                ui.label(format!("CFO: {:.1} Hz", ch_config.cfo_hz));
            });

            // RX column
            columns[2].group(|ui| {
                ui.heading("Received");
                ui.add_space(4.0);

                if let Some(ref rx) = self.rx_payload {
                    // Try to decode as UTF-8
                    match String::from_utf8(rx.clone()) {
                        Ok(s) => {
                            ui.label("Decoded:");
                            ui.code(&s);
                        }
                        Err(_) => {
                            ui.label("Raw bytes:");
                            let hex: String = rx.iter().map(|b| format!("{:02X} ", b)).collect();
                            ui.code(&hex);
                        }
                    }

                    if let Some((matched, total)) = self.symbols_match {
                        ui.add_space(8.0);
                        let percent = matched as f64 / total as f64 * 100.0;
                        ui.label(format!("Symbol match: {}/{} ({:.1}%)", matched, total, percent));

                        // Color indicator
                        let color = if percent > 99.0 {
                            egui::Color32::GREEN
                        } else if percent > 80.0 {
                            egui::Color32::YELLOW
                        } else {
                            egui::Color32::RED
                        };

                        ui.colored_label(
                            color,
                            if percent > 99.0 {
                                "Excellent!"
                            } else if percent > 80.0 {
                                "Good"
                            } else {
                                "Errors detected"
                            },
                        );
                    }
                } else {
                    ui.label("Click 'Run Full Pipeline' to decode");
                }
            });
        });

        ui.add_space(20.0);
        ui.separator();

        // Tips
        ui.collapsing("Experiment Tips", |ui| {
            ui.label("• Lower SNR to see how errors increase");
            ui.label("• Add CFO to see synchronization challenges");
            ui.label("• Try different spreading factors for range/speed tradeoffs");
            ui.label("• Higher coding rates provide more error correction");
        });
    }
}
