//! # R4W Example Waveform Plugin
//!
//! This is an example plugin demonstrating how to create waveform plugins
//! for the R4W SDR framework.
//!
//! ## Building
//!
//! ```bash
//! cargo build --release -p r4w-example-plugin
//! ```
//!
//! This produces `target/release/libr4w_example_plugin.so` (Linux)
//! or `.dll` (Windows) or `.dylib` (macOS).
//!
//! ## Loading
//!
//! ```rust,ignore
//! use r4w_core::plugin::PluginManager;
//!
//! let mut manager = PluginManager::new();
//! manager.add_search_path("./target/release");
//! manager.discover_plugins()?;
//! ```
//!
//! trace:FR-0096 | ai:claude

use std::ffi::{c_char, c_void};

use r4w_core::plugin::{
    PluginInfo, WaveformDescriptor, PLUGIN_API_VERSION,
    caps::{CAN_MODULATE, CAN_DEMODULATE},
};
use r4w_core::types::IQSample;
use r4w_core::waveform::{CommonParams, DemodResult, Waveform, WaveformInfo};

// ============================================================================
// Plugin Metadata
// ============================================================================

/// Plugin name as C string
static PLUGIN_NAME: &[u8] = b"example_waveform\0";
/// Plugin version
static PLUGIN_VERSION: &[u8] = b"1.0.0\0";
/// Plugin description
static PLUGIN_DESCRIPTION: &[u8] = b"Example waveform plugin demonstrating plugin API\0";
/// Plugin author
static PLUGIN_AUTHOR: &[u8] = b"R4W Project\0";

/// Static plugin info
static PLUGIN_INFO: PluginInfo = PluginInfo {
    name: PLUGIN_NAME.as_ptr() as *const c_char,
    version: PLUGIN_VERSION.as_ptr() as *const c_char,
    description: PLUGIN_DESCRIPTION.as_ptr() as *const c_char,
    author: PLUGIN_AUTHOR.as_ptr() as *const c_char,
    waveform_count: 1,
};

// ============================================================================
// Waveform Descriptors
// ============================================================================

/// Pulse waveform ID
static PULSE_ID: &[u8] = b"pulse\0";
/// Pulse waveform name
static PULSE_NAME: &[u8] = b"Pulse Modulation\0";
/// Pulse waveform description
static PULSE_DESCRIPTION: &[u8] = b"Simple pulse modulation for demonstration\0";

/// Static waveform descriptors
static WAVEFORM_DESCRIPTORS: [WaveformDescriptor; 1] = [
    WaveformDescriptor {
        id: PULSE_ID.as_ptr() as *const c_char,
        name: PULSE_NAME.as_ptr() as *const c_char,
        description: PULSE_DESCRIPTION.as_ptr() as *const c_char,
        min_sample_rate: 1000.0,
        max_sample_rate: 100_000_000.0,
        capabilities: CAN_MODULATE | CAN_DEMODULATE,
    },
];

// ============================================================================
// Plugin ABI Functions
// ============================================================================

/// Return the plugin API version for compatibility checking.
#[no_mangle]
pub extern "C" fn r4w_plugin_api_version() -> u32 {
    PLUGIN_API_VERSION
}

/// Return plugin metadata.
#[no_mangle]
pub extern "C" fn r4w_plugin_info() -> *const PluginInfo {
    &PLUGIN_INFO
}

/// List available waveforms in this plugin.
#[no_mangle]
pub extern "C" fn r4w_list_waveforms(count: *mut u32) -> *const WaveformDescriptor {
    if !count.is_null() {
        unsafe { *count = WAVEFORM_DESCRIPTORS.len() as u32 };
    }
    WAVEFORM_DESCRIPTORS.as_ptr()
}

/// Create a waveform instance.
///
/// # Safety
///
/// The returned handle must be passed to `r4w_destroy_waveform` when no longer needed.
#[no_mangle]
pub extern "C" fn r4w_create_waveform(
    id: *const c_char,
    sample_rate: f64,
) -> *mut c_void {
    if id.is_null() {
        return std::ptr::null_mut();
    }

    let id_cstr = unsafe { std::ffi::CStr::from_ptr(id) };
    let id_str = match id_cstr.to_str() {
        Ok(s) => s,
        Err(_) => return std::ptr::null_mut(),
    };

    match id_str {
        "pulse" => {
            let common = CommonParams {
                sample_rate,
                carrier_freq: 0.0,
                amplitude: 1.0,
            };
            let waveform = Box::new(PulseWaveform::new(common));
            Box::into_raw(waveform) as *mut c_void
        }
        _ => std::ptr::null_mut(),
    }
}

/// Destroy a waveform instance.
///
/// # Safety
///
/// The handle must have been created by `r4w_create_waveform` and not already destroyed.
#[no_mangle]
pub extern "C" fn r4w_destroy_waveform(handle: *mut c_void) {
    if !handle.is_null() {
        unsafe {
            let _ = Box::from_raw(handle as *mut PulseWaveform);
        }
    }
}

// ============================================================================
// Pulse Waveform Implementation
// ============================================================================

/// Simple pulse modulation waveform for demonstration.
///
/// This is a minimal waveform that converts each bit to a pulse:
/// - Bit 1 = positive pulse (+1.0)
/// - Bit 0 = negative pulse (-1.0)
///
/// This is primarily to demonstrate the plugin architecture.
#[derive(Debug, Clone)]
pub struct PulseWaveform {
    common: CommonParams,
    samples_per_symbol: usize,
}

impl PulseWaveform {
    /// Create a new pulse waveform.
    pub fn new(common: CommonParams) -> Self {
        Self {
            common,
            samples_per_symbol: 10,
        }
    }
}

impl Waveform for PulseWaveform {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "Pulse",
            full_name: "Pulse Modulation",
            description: "Simple +1/-1 pulses for demonstration",
            complexity: 1,
            bits_per_symbol: 1,
            carries_data: true,
            characteristics: &[
                "Simplest baseband modulation",
                "1 bit per symbol",
                "+1 = 1, -1 = 0",
            ],
            history: "Pulse modulation is the foundation of digital communications.",
            modern_usage: "Used as a building block for more complex schemes.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn samples_per_symbol(&self) -> usize {
        self.samples_per_symbol
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        let mut samples = Vec::new();

        for byte in data {
            for bit_idx in (0..8).rev() {
                let bit = (byte >> bit_idx) & 1;
                let amplitude = if bit == 1 { 1.0 } else { -1.0 };

                for _ in 0..self.samples_per_symbol {
                    samples.push(IQSample::new(amplitude * self.common.amplitude, 0.0));
                }
            }
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut result = DemodResult::default();
        let mut current_byte = 0u8;
        let mut bit_count = 0;

        for chunk in samples.chunks(self.samples_per_symbol) {
            // Average the real part
            let avg: f64 = chunk.iter().map(|s| s.re).sum::<f64>() / chunk.len() as f64;

            // Decision: positive = 1, negative = 0
            let bit = if avg > 0.0 { 1u8 } else { 0u8 };
            result.symbols.push(bit as u16);

            current_byte = (current_byte << 1) | bit;
            bit_count += 1;

            if bit_count == 8 {
                result.bits.push(current_byte);
                current_byte = 0;
                bit_count = 0;
            }
        }

        result
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plugin_api_version() {
        assert_eq!(r4w_plugin_api_version(), PLUGIN_API_VERSION);
    }

    #[test]
    fn test_plugin_info() {
        let info = unsafe { &*r4w_plugin_info() };
        assert_eq!(info.waveform_count, 1);
    }

    #[test]
    fn test_list_waveforms() {
        let mut count = 0u32;
        let descriptors = r4w_list_waveforms(&mut count);
        assert_eq!(count, 1);
        assert!(!descriptors.is_null());
    }

    #[test]
    fn test_pulse_roundtrip() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let waveform = PulseWaveform::new(common);

        let data = b"Hi";
        let modulated = waveform.modulate(data);
        let result = waveform.demodulate(&modulated);

        assert_eq!(&result.bits, data);
    }

    #[test]
    fn test_create_destroy_waveform() {
        let id = b"pulse\0";
        let handle = r4w_create_waveform(id.as_ptr() as *const c_char, 10000.0);
        assert!(!handle.is_null());
        r4w_destroy_waveform(handle);
    }

    #[test]
    fn test_create_invalid_waveform() {
        let id = b"invalid\0";
        let handle = r4w_create_waveform(id.as_ptr() as *const c_char, 10000.0);
        assert!(handle.is_null());
    }
}
