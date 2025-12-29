//! R4W Explorer - WASM Entry Point
//!
//! This crate provides the WebAssembly entry point for R4W (Rust for Waveforms).
//! It initializes the eframe web runner and starts the application in the browser.
//!
//! This crate only compiles for wasm32 target.

#![cfg(target_arch = "wasm32")]

use wasm_bindgen::prelude::*;

/// Main entry point for the WASM application
#[wasm_bindgen(start)]
pub fn start() -> Result<(), JsValue> {
    // Set up panic hook for better error messages
    console_error_panic_hook::set_once();

    // Initialize logging to browser console
    console_log::init_with_level(log::Level::Debug)
        .expect("Failed to initialize logger");

    log::info!("R4W Explorer starting...");

    // Spawn the async initialization
    wasm_bindgen_futures::spawn_local(async {
        if let Err(e) = run_app().await {
            log::error!("Failed to start application: {:?}", e);
        }
    });

    Ok(())
}

async fn run_app() -> Result<(), wasm_bindgen::JsValue> {
    // Get the canvas element
    let canvas = create_canvas()?;

    // Configure eframe for web
    let web_options = eframe::WebOptions::default();

    // Start the application
    eframe::WebRunner::new()
        .start(
            canvas,
            web_options,
            Box::new(|cc| Ok(Box::new(r4w_gui::WaveformExplorer::new(cc)))),
        )
        .await?;

    // Hide the loading indicator
    hide_loading();

    log::info!("R4W Explorer started successfully!");

    Ok(())
}

fn create_canvas() -> Result<web_sys::HtmlCanvasElement, JsValue> {
    let document = web_sys::window()
        .ok_or_else(|| JsValue::from_str("No window found"))?
        .document()
        .ok_or_else(|| JsValue::from_str("No document found"))?;

    let canvas = document
        .create_element("canvas")?;

    let canvas: web_sys::HtmlCanvasElement = canvas
        .dyn_into()
        .map_err(|_| JsValue::from_str("Failed to cast to canvas"))?;

    canvas.set_id("canvas");

    // Add canvas to body
    document
        .body()
        .ok_or_else(|| JsValue::from_str("No body found"))?
        .append_child(&canvas)?;

    Ok(canvas)
}

fn hide_loading() {
    if let Some(el) = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.get_element_by_id("loading"))
    {
        let _ = el.set_attribute("style", "display:none");
    }
}
