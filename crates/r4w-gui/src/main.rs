//! SDR Waveform Explorer (Native Entry Point)
//!
//! An interactive application for learning about Software Defined Radio
//! and digital modulation techniques including AM, FM, PSK, QAM, OFDM, and more.
//!
//! This is the native (desktop) entry point. For web/WASM, see the
//! `r4w-gui-web` crate.

use r4w_gui::WaveformExplorer;

fn main() -> eframe::Result<()> {
    // Initialize logging (native only)
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([800.0, 600.0])
            .with_title("R4W - SDR Developer Studio"),
        ..Default::default()
    };

    eframe::run_native(
        "Waveform Explorer",
        native_options,
        Box::new(|cc| Ok(Box::new(WaveformExplorer::new(cc)))),
    )
}
