//! # RTL-SDR FFI Bindings
//!
//! Low-level FFI bindings to librtlsdr for RTL2832U-based SDR devices.
//!
//! This module uses dynamic loading (libloading) to avoid compile-time
//! dependency on librtlsdr. The library is loaded at runtime, allowing
//! the same binary to work with or without RTL-SDR hardware.
//!
//! ## Library Functions
//!
//! The following librtlsdr functions are wrapped:
//! - `rtlsdr_get_device_count` - Get number of connected devices
//! - `rtlsdr_get_device_name` - Get device name by index
//! - `rtlsdr_get_device_usb_strings` - Get USB vendor/product/serial
//! - `rtlsdr_open` / `rtlsdr_close` - Open/close device
//! - `rtlsdr_set_center_freq` / `rtlsdr_get_center_freq` - Frequency control
//! - `rtlsdr_set_sample_rate` / `rtlsdr_get_sample_rate` - Sample rate
//! - `rtlsdr_set_tuner_gain` / `rtlsdr_get_tuner_gain` - Gain control
//! - `rtlsdr_set_tuner_gain_mode` - Manual/auto gain
//! - `rtlsdr_read_sync` - Synchronous sample reading
//! - `rtlsdr_reset_buffer` - Reset the internal buffer
//!
//! ## Sample Format
//!
//! RTL-SDR outputs 8-bit unsigned I/Q samples:
//! - I and Q interleaved: [I0, Q0, I1, Q1, ...]
//! - Value range: 0-255 (128 = zero)
//! - Conversion: (sample - 127.5) / 127.5 → normalized f64

use std::ffi::{c_char, c_int, c_uint, c_void, CStr};
use std::ptr;
use std::sync::OnceLock;

use libloading::{Library, Symbol};

/// RTL-SDR device handle (opaque pointer).
pub type RtlSdrDevHandle = *mut c_void;

/// Result type for RTL-SDR operations.
pub type RtlSdrResult<T> = Result<T, RtlSdrError>;

/// RTL-SDR error types.
#[derive(Debug, Clone, thiserror::Error)]
pub enum RtlSdrError {
    #[error("librtlsdr not found - install rtl-sdr package")]
    LibraryNotFound,

    #[error("Device not found: {0}")]
    DeviceNotFound(String),

    #[error("Device open failed: error code {0}")]
    OpenFailed(i32),

    #[error("Operation failed: {0} (error code {1})")]
    OperationFailed(String, i32),

    #[error("Read failed: got {got} bytes, expected {expected}")]
    ReadFailed { expected: usize, got: usize },

    #[error("Invalid device handle")]
    InvalidHandle,

    #[error("Device busy")]
    DeviceBusy,
}

/// Loaded librtlsdr library and function pointers.
struct RtlSdrLib {
    _lib: Library,
    get_device_count: Symbol<'static, unsafe extern "C" fn() -> c_uint>,
    get_device_name: Symbol<'static, unsafe extern "C" fn(c_uint) -> *const c_char>,
    get_device_usb_strings: Symbol<
        'static,
        unsafe extern "C" fn(c_uint, *mut c_char, *mut c_char, *mut c_char) -> c_int,
    >,
    open: Symbol<'static, unsafe extern "C" fn(*mut RtlSdrDevHandle, c_uint) -> c_int>,
    close: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle) -> c_int>,
    set_center_freq: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle, c_uint) -> c_int>,
    get_center_freq: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle) -> c_uint>,
    set_sample_rate: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle, c_uint) -> c_int>,
    get_sample_rate: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle) -> c_uint>,
    set_tuner_gain_mode: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle, c_int) -> c_int>,
    set_tuner_gain: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle, c_int) -> c_int>,
    get_tuner_gain: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle) -> c_int>,
    get_tuner_gains: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle, *mut c_int) -> c_int>,
    set_agc_mode: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle, c_int) -> c_int>,
    set_direct_sampling: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle, c_int) -> c_int>,
    set_freq_correction: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle, c_int) -> c_int>,
    set_tuner_bandwidth: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle, c_uint) -> c_int>,
    reset_buffer: Symbol<'static, unsafe extern "C" fn(RtlSdrDevHandle) -> c_int>,
    read_sync: Symbol<
        'static,
        unsafe extern "C" fn(RtlSdrDevHandle, *mut c_void, c_int, *mut c_int) -> c_int,
    >,
}

/// Global library instance (loaded once).
static RTLSDR_LIB: OnceLock<Option<RtlSdrLib>> = OnceLock::new();

/// Library names to try on different platforms.
#[cfg(target_os = "linux")]
const LIB_NAMES: &[&str] = &["librtlsdr.so.0", "librtlsdr.so"];

#[cfg(target_os = "macos")]
const LIB_NAMES: &[&str] = &["librtlsdr.dylib", "librtlsdr.0.dylib"];

#[cfg(target_os = "windows")]
const LIB_NAMES: &[&str] = &["rtlsdr.dll", "librtlsdr.dll"];

/// Load the RTL-SDR library.
fn load_library() -> Option<RtlSdrLib> {
    for name in LIB_NAMES {
        if let Ok(lib) = unsafe { Library::new(name) } {
            // Try to load all required symbols
            let result = unsafe {
                // We need to transmute the library reference to 'static for the symbols.
                // This is safe because we keep the Library alive in the struct.
                let lib_ref: &'static Library = std::mem::transmute(&lib);

                Some(RtlSdrLib {
                    get_device_count: lib_ref.get(b"rtlsdr_get_device_count\0").ok()?,
                    get_device_name: lib_ref.get(b"rtlsdr_get_device_name\0").ok()?,
                    get_device_usb_strings: lib_ref.get(b"rtlsdr_get_device_usb_strings\0").ok()?,
                    open: lib_ref.get(b"rtlsdr_open\0").ok()?,
                    close: lib_ref.get(b"rtlsdr_close\0").ok()?,
                    set_center_freq: lib_ref.get(b"rtlsdr_set_center_freq\0").ok()?,
                    get_center_freq: lib_ref.get(b"rtlsdr_get_center_freq\0").ok()?,
                    set_sample_rate: lib_ref.get(b"rtlsdr_set_sample_rate\0").ok()?,
                    get_sample_rate: lib_ref.get(b"rtlsdr_get_sample_rate\0").ok()?,
                    set_tuner_gain_mode: lib_ref.get(b"rtlsdr_set_tuner_gain_mode\0").ok()?,
                    set_tuner_gain: lib_ref.get(b"rtlsdr_set_tuner_gain\0").ok()?,
                    get_tuner_gain: lib_ref.get(b"rtlsdr_get_tuner_gain\0").ok()?,
                    get_tuner_gains: lib_ref.get(b"rtlsdr_get_tuner_gains\0").ok()?,
                    set_agc_mode: lib_ref.get(b"rtlsdr_set_agc_mode\0").ok()?,
                    set_direct_sampling: lib_ref.get(b"rtlsdr_set_direct_sampling\0").ok()?,
                    set_freq_correction: lib_ref.get(b"rtlsdr_set_freq_correction\0").ok()?,
                    set_tuner_bandwidth: lib_ref.get(b"rtlsdr_set_tuner_bandwidth\0").ok()?,
                    reset_buffer: lib_ref.get(b"rtlsdr_reset_buffer\0").ok()?,
                    read_sync: lib_ref.get(b"rtlsdr_read_sync\0").ok()?,
                    _lib: lib,
                })
            };

            if result.is_some() {
                tracing::info!("Loaded RTL-SDR library: {}", name);
                return result;
            }
        }
    }
    tracing::debug!("RTL-SDR library not found");
    None
}

/// Get the loaded library, initializing if necessary.
fn get_lib() -> Option<&'static RtlSdrLib> {
    RTLSDR_LIB.get_or_init(load_library).as_ref()
}

/// Check if librtlsdr is available.
pub fn is_available() -> bool {
    get_lib().is_some()
}

/// Get the number of connected RTL-SDR devices.
pub fn get_device_count() -> u32 {
    get_lib()
        .map(|lib| unsafe { (lib.get_device_count)() })
        .unwrap_or(0)
}

/// Get device name by index.
pub fn get_device_name(index: u32) -> Option<String> {
    get_lib().and_then(|lib| {
        let name_ptr = unsafe { (lib.get_device_name)(index) };
        if name_ptr.is_null() {
            None
        } else {
            Some(unsafe { CStr::from_ptr(name_ptr).to_string_lossy().into_owned() })
        }
    })
}

/// Device USB strings (manufacturer, product, serial).
#[derive(Debug, Clone)]
pub struct UsbStrings {
    pub manufacturer: String,
    pub product: String,
    pub serial: String,
}

/// Get USB strings for a device.
pub fn get_device_usb_strings(index: u32) -> Option<UsbStrings> {
    get_lib().and_then(|lib| {
        let mut manufact = [0u8; 256];
        let mut product = [0u8; 256];
        let mut serial = [0u8; 256];

        let ret = unsafe {
            (lib.get_device_usb_strings)(
                index,
                manufact.as_mut_ptr() as *mut c_char,
                product.as_mut_ptr() as *mut c_char,
                serial.as_mut_ptr() as *mut c_char,
            )
        };

        if ret == 0 {
            Some(UsbStrings {
                manufacturer: String::from_utf8_lossy(
                    &manufact[..manufact.iter().position(|&x| x == 0).unwrap_or(0)],
                )
                .into_owned(),
                product: String::from_utf8_lossy(
                    &product[..product.iter().position(|&x| x == 0).unwrap_or(0)],
                )
                .into_owned(),
                serial: String::from_utf8_lossy(
                    &serial[..serial.iter().position(|&x| x == 0).unwrap_or(0)],
                )
                .into_owned(),
            })
        } else {
            None
        }
    })
}

/// Safe wrapper around an open RTL-SDR device.
pub struct RtlSdrHandle {
    handle: RtlSdrDevHandle,
    index: u32,
    available_gains: Vec<i32>,
}

// SAFETY: RTL-SDR handle is thread-safe for read operations.
// We only do synchronous reads which are internally locked.
unsafe impl Send for RtlSdrHandle {}

impl RtlSdrHandle {
    /// Open an RTL-SDR device by index.
    pub fn open(index: u32) -> RtlSdrResult<Self> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;

        let mut handle: RtlSdrDevHandle = ptr::null_mut();
        let ret = unsafe { (lib.open)(&mut handle, index) };

        if ret != 0 {
            return Err(RtlSdrError::OpenFailed(ret));
        }

        if handle.is_null() {
            return Err(RtlSdrError::InvalidHandle);
        }

        // Query available gains
        let mut gains = [0i32; 64];
        let num_gains = unsafe { (lib.get_tuner_gains)(handle, gains.as_mut_ptr()) };
        let available_gains = if num_gains > 0 {
            gains[..num_gains as usize].to_vec()
        } else {
            Vec::new()
        };

        tracing::info!(
            "Opened RTL-SDR device #{} with {} gain levels",
            index,
            available_gains.len()
        );

        Ok(Self {
            handle,
            index,
            available_gains,
        })
    }

    /// Get the device index.
    pub fn index(&self) -> u32 {
        self.index
    }

    /// Get available gain values (in tenths of dB).
    pub fn available_gains(&self) -> &[i32] {
        &self.available_gains
    }

    /// Find the nearest valid gain value.
    pub fn nearest_gain(&self, gain_tenth_db: i32) -> i32 {
        self.available_gains
            .iter()
            .min_by_key(|&&g| (g - gain_tenth_db).abs())
            .copied()
            .unwrap_or(0)
    }

    /// Set center frequency in Hz.
    pub fn set_center_freq(&mut self, freq_hz: u32) -> RtlSdrResult<()> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.set_center_freq)(self.handle, freq_hz) };
        if ret != 0 {
            Err(RtlSdrError::OperationFailed(
                "set_center_freq".to_string(),
                ret,
            ))
        } else {
            Ok(())
        }
    }

    /// Get current center frequency in Hz.
    pub fn get_center_freq(&self) -> u32 {
        get_lib()
            .map(|lib| unsafe { (lib.get_center_freq)(self.handle) })
            .unwrap_or(0)
    }

    /// Set sample rate in Hz.
    pub fn set_sample_rate(&mut self, rate_hz: u32) -> RtlSdrResult<()> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.set_sample_rate)(self.handle, rate_hz) };
        if ret != 0 {
            Err(RtlSdrError::OperationFailed(
                "set_sample_rate".to_string(),
                ret,
            ))
        } else {
            Ok(())
        }
    }

    /// Get current sample rate in Hz.
    pub fn get_sample_rate(&self) -> u32 {
        get_lib()
            .map(|lib| unsafe { (lib.get_sample_rate)(self.handle) })
            .unwrap_or(0)
    }

    /// Set tuner gain mode (0 = auto, 1 = manual).
    pub fn set_tuner_gain_mode(&mut self, manual: bool) -> RtlSdrResult<()> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.set_tuner_gain_mode)(self.handle, if manual { 1 } else { 0 }) };
        if ret != 0 {
            Err(RtlSdrError::OperationFailed(
                "set_tuner_gain_mode".to_string(),
                ret,
            ))
        } else {
            Ok(())
        }
    }

    /// Set tuner gain in tenths of dB.
    ///
    /// The gain is rounded to the nearest available value.
    pub fn set_tuner_gain(&mut self, gain_tenth_db: i32) -> RtlSdrResult<i32> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;

        // Find nearest valid gain
        let actual_gain = self.nearest_gain(gain_tenth_db);

        let ret = unsafe { (lib.set_tuner_gain)(self.handle, actual_gain) };
        if ret != 0 {
            Err(RtlSdrError::OperationFailed(
                "set_tuner_gain".to_string(),
                ret,
            ))
        } else {
            Ok(actual_gain)
        }
    }

    /// Get current tuner gain in tenths of dB.
    pub fn get_tuner_gain(&self) -> i32 {
        get_lib()
            .map(|lib| unsafe { (lib.get_tuner_gain)(self.handle) })
            .unwrap_or(0)
    }

    /// Set RTL2832 AGC mode (0 = off, 1 = on).
    pub fn set_agc_mode(&mut self, on: bool) -> RtlSdrResult<()> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.set_agc_mode)(self.handle, if on { 1 } else { 0 }) };
        if ret != 0 {
            Err(RtlSdrError::OperationFailed("set_agc_mode".to_string(), ret))
        } else {
            Ok(())
        }
    }

    /// Set direct sampling mode (0 = off, 1 = I-ADC, 2 = Q-ADC).
    pub fn set_direct_sampling(&mut self, mode: i32) -> RtlSdrResult<()> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.set_direct_sampling)(self.handle, mode) };
        if ret != 0 {
            Err(RtlSdrError::OperationFailed(
                "set_direct_sampling".to_string(),
                ret,
            ))
        } else {
            Ok(())
        }
    }

    /// Set frequency correction in PPM.
    pub fn set_freq_correction(&mut self, ppm: i32) -> RtlSdrResult<()> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.set_freq_correction)(self.handle, ppm) };
        if ret != 0 {
            Err(RtlSdrError::OperationFailed(
                "set_freq_correction".to_string(),
                ret,
            ))
        } else {
            Ok(())
        }
    }

    /// Set tuner bandwidth in Hz (0 = automatic).
    pub fn set_tuner_bandwidth(&mut self, bw_hz: u32) -> RtlSdrResult<()> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.set_tuner_bandwidth)(self.handle, bw_hz) };
        if ret != 0 {
            Err(RtlSdrError::OperationFailed(
                "set_tuner_bandwidth".to_string(),
                ret,
            ))
        } else {
            Ok(())
        }
    }

    /// Reset the streaming buffer.
    pub fn reset_buffer(&mut self) -> RtlSdrResult<()> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.reset_buffer)(self.handle) };
        if ret != 0 {
            Err(RtlSdrError::OperationFailed(
                "reset_buffer".to_string(),
                ret,
            ))
        } else {
            Ok(())
        }
    }

    /// Read samples synchronously.
    ///
    /// Returns raw 8-bit unsigned I/Q samples.
    /// Buffer length must be a multiple of 512.
    ///
    /// # Arguments
    /// * `buffer` - Buffer to fill with raw samples (I0, Q0, I1, Q1, ...)
    ///
    /// # Returns
    /// Number of bytes actually read.
    pub fn read_sync(&mut self, buffer: &mut [u8]) -> RtlSdrResult<usize> {
        let lib = get_lib().ok_or(RtlSdrError::LibraryNotFound)?;

        let mut n_read: c_int = 0;
        let ret = unsafe {
            (lib.read_sync)(
                self.handle,
                buffer.as_mut_ptr() as *mut c_void,
                buffer.len() as c_int,
                &mut n_read,
            )
        };

        if ret != 0 {
            Err(RtlSdrError::OperationFailed("read_sync".to_string(), ret))
        } else {
            Ok(n_read as usize)
        }
    }
}

impl Drop for RtlSdrHandle {
    fn drop(&mut self) {
        if let Some(lib) = get_lib() {
            tracing::debug!("Closing RTL-SDR device #{}", self.index);
            unsafe { (lib.close)(self.handle) };
        }
    }
}

/// Convert raw 8-bit unsigned samples to normalized f64.
///
/// RTL-SDR outputs unsigned 8-bit samples where:
/// - 0 = -1.0
/// - 128 = 0.0
/// - 255 = ~+1.0
///
/// This function converts to the range [-1.0, 1.0].
#[inline]
pub fn u8_to_f64(sample: u8) -> f64 {
    (sample as f64 - 127.5) / 127.5
}

/// Convert a buffer of raw I/Q samples to complex f64.
///
/// Input format: [I0, Q0, I1, Q1, ...]
/// Output: Vec of complex samples
pub fn convert_samples(raw: &[u8]) -> Vec<num_complex::Complex64> {
    raw.chunks_exact(2)
        .map(|chunk| num_complex::Complex64::new(u8_to_f64(chunk[0]), u8_to_f64(chunk[1])))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_u8_to_f64_conversion() {
        // Zero point
        let mid = u8_to_f64(128);
        assert!((mid - 0.0).abs() < 0.01);

        // Min value
        let min = u8_to_f64(0);
        assert!((min - (-1.0)).abs() < 0.01);

        // Max value
        let max = u8_to_f64(255);
        assert!((max - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_convert_samples() {
        let raw = [128u8, 128, 0, 255, 255, 0];
        let samples = convert_samples(&raw);

        assert_eq!(samples.len(), 3);

        // First sample: (128, 128) → (0, 0)
        assert!((samples[0].re - 0.0).abs() < 0.01);
        assert!((samples[0].im - 0.0).abs() < 0.01);

        // Second sample: (0, 255) → (-1, 1)
        assert!((samples[1].re - (-1.0)).abs() < 0.01);
        assert!((samples[1].im - 1.0).abs() < 0.01);

        // Third sample: (255, 0) → (1, -1)
        assert!((samples[2].re - 1.0).abs() < 0.01);
        assert!((samples[2].im - (-1.0)).abs() < 0.01);
    }

    #[test]
    fn test_library_availability() {
        // This test just checks if the library detection works
        let available = is_available();
        let count = get_device_count();

        if available {
            println!("librtlsdr is available, {} devices found", count);
        } else {
            println!("librtlsdr not available (expected on most dev machines)");
        }
    }
}
