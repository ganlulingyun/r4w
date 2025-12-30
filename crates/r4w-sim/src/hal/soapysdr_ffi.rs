//! # SoapySDR FFI Bindings
//!
//! Low-level FFI bindings to libSoapySDR for vendor-neutral SDR access.
//!
//! This module uses dynamic loading (libloading) to avoid compile-time
//! dependency on libSoapySDR. The library is loaded at runtime, allowing
//! the same binary to work with or without SoapySDR hardware.
//!
//! ## Supported Hardware
//!
//! Through SoapySDR plugins:
//! - **LimeSDR**: Full-duplex transceiver, 100 kHz - 3.8 GHz
//! - **HackRF**: Half-duplex, 1 MHz - 6 GHz
//! - **PlutoSDR**: AD9361-based, 325 MHz - 3.8 GHz
//! - **BladeRF**: 300 MHz - 3.8 GHz
//! - **Airspy**: RX only, 24 - 1800 MHz
//! - **RTL-SDR**: via soapy-rtlsdr plugin
//! - **USRP**: via soapy-uhd plugin
//!
//! ## Sample Format
//!
//! SoapySDR supports multiple sample formats. We request CF32 (complex float32)
//! which matches our native IQSample format.

use std::collections::HashMap;
use std::ffi::{c_char, c_double, c_int, c_void, CStr, CString};
use std::ptr;
use std::sync::OnceLock;

use libloading::{Library, Symbol};

/// SoapySDR device handle (opaque pointer).
pub type SoapyDeviceHandle = *mut c_void;

/// SoapySDR stream handle (opaque pointer).
pub type SoapyStreamHandle = *mut c_void;

/// SoapySDR kwargs structure (key-value pairs).
#[repr(C)]
pub struct SoapySDRKwargs {
    pub size: usize,
    pub keys: *mut *mut c_char,
    pub vals: *mut *mut c_char,
}

/// SoapySDR range structure.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct SoapySDRRange {
    pub minimum: c_double,
    pub maximum: c_double,
    pub step: c_double,
}

/// Stream direction constants.
pub const SOAPY_SDR_RX: c_int = 0;
pub const SOAPY_SDR_TX: c_int = 1;

/// Stream format string for complex float32.
pub const SOAPY_SDR_CF32: &[u8] = b"CF32\0";

/// Result type for SoapySDR operations.
pub type SoapySdrResult<T> = Result<T, SoapySdrError>;

/// SoapySDR error types.
#[derive(Debug, Clone, thiserror::Error)]
pub enum SoapySdrError {
    #[error("libSoapySDR not found - install SoapySDR package")]
    LibraryNotFound,

    #[error("Device not found: {0}")]
    DeviceNotFound(String),

    #[error("Failed to create device: {0}")]
    CreateFailed(String),

    #[error("Stream setup failed: {0}")]
    StreamSetupFailed(String),

    #[error("Operation failed: {0}")]
    OperationFailed(String),

    #[error("Read failed: {0}")]
    ReadFailed(String),

    #[error("Write failed: {0}")]
    WriteFailed(String),

    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),
}

/// Loaded libSoapySDR library and function pointers.
struct SoapySdrLib {
    _lib: Library,
    // Device enumeration
    enumerate: Symbol<'static, unsafe extern "C" fn(*const SoapySDRKwargs) -> *mut SoapySDRKwargs>,
    enumerate_free: Symbol<'static, unsafe extern "C" fn(*mut SoapySDRKwargs, usize)>,
    // Device creation
    make: Symbol<'static, unsafe extern "C" fn(*const SoapySDRKwargs) -> SoapyDeviceHandle>,
    unmake: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle) -> c_int>,
    // Device info
    get_hardware_key: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle) -> *mut c_char>,
    get_hardware_info: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle) -> SoapySDRKwargs>,
    // Channel info
    get_num_channels: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int) -> usize>,
    // Frequency
    set_frequency: Symbol<
        'static,
        unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize, c_double, *const SoapySDRKwargs) -> c_int,
    >,
    get_frequency: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize) -> c_double>,
    get_frequency_range: Symbol<
        'static,
        unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize, *mut usize) -> *mut SoapySDRRange,
    >,
    // Sample rate
    set_sample_rate: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize, c_double) -> c_int>,
    get_sample_rate: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize) -> c_double>,
    get_sample_rate_range: Symbol<
        'static,
        unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize, *mut usize) -> *mut SoapySDRRange,
    >,
    // Bandwidth
    set_bandwidth: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize, c_double) -> c_int>,
    get_bandwidth: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize) -> c_double>,
    // Gain
    set_gain: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize, c_double) -> c_int>,
    get_gain: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize) -> c_double>,
    get_gain_range: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize) -> SoapySDRRange>,
    // Antenna
    set_antenna: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize, *const c_char) -> c_int>,
    get_antenna: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize) -> *mut c_char>,
    list_antennas: Symbol<
        'static,
        unsafe extern "C" fn(SoapyDeviceHandle, c_int, usize, *mut usize) -> *mut *mut c_char,
    >,
    // Streaming
    setup_stream: Symbol<
        'static,
        unsafe extern "C" fn(
            SoapyDeviceHandle,
            c_int,
            *const c_char,
            *const usize,
            usize,
            *const SoapySDRKwargs,
        ) -> SoapyStreamHandle,
    >,
    close_stream: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, SoapyStreamHandle) -> c_int>,
    get_stream_mtu: Symbol<'static, unsafe extern "C" fn(SoapyDeviceHandle, SoapyStreamHandle) -> usize>,
    activate_stream: Symbol<
        'static,
        unsafe extern "C" fn(SoapyDeviceHandle, SoapyStreamHandle, c_int, i64, usize) -> c_int,
    >,
    deactivate_stream: Symbol<
        'static,
        unsafe extern "C" fn(SoapyDeviceHandle, SoapyStreamHandle, c_int, i64) -> c_int,
    >,
    read_stream: Symbol<
        'static,
        unsafe extern "C" fn(
            SoapyDeviceHandle,
            SoapyStreamHandle,
            *mut *mut c_void,
            usize,
            *mut c_int,
            *mut i64,
            i64,
        ) -> c_int,
    >,
    write_stream: Symbol<
        'static,
        unsafe extern "C" fn(
            SoapyDeviceHandle,
            SoapyStreamHandle,
            *const *const c_void,
            usize,
            *mut c_int,
            i64,
            i64,
        ) -> c_int,
    >,
    // String list free
    strings_free: Symbol<'static, unsafe extern "C" fn(*mut *mut c_char, usize)>,
    // Kwargs
    kwargs_clear: Symbol<'static, unsafe extern "C" fn(*mut SoapySDRKwargs)>,
    kwargs_set: Symbol<'static, unsafe extern "C" fn(*mut SoapySDRKwargs, *const c_char, *const c_char)>,
    // Last error
    last_error: Symbol<'static, unsafe extern "C" fn() -> *const c_char>,
}

/// Global library instance (loaded once).
static SOAPYSDR_LIB: OnceLock<Option<SoapySdrLib>> = OnceLock::new();

/// Library names to try on different platforms.
#[cfg(target_os = "linux")]
const LIB_NAMES: &[&str] = &["libSoapySDR.so.0.8", "libSoapySDR.so.0.7", "libSoapySDR.so"];

#[cfg(target_os = "macos")]
const LIB_NAMES: &[&str] = &["libSoapySDR.dylib", "libSoapySDR.0.8.dylib"];

#[cfg(target_os = "windows")]
const LIB_NAMES: &[&str] = &["SoapySDR.dll", "libSoapySDR.dll"];

/// Load the SoapySDR library.
fn load_library() -> Option<SoapySdrLib> {
    for name in LIB_NAMES {
        if let Ok(lib) = unsafe { Library::new(name) } {
            // Try to load all required symbols
            let result = unsafe {
                // We need to transmute the library reference to 'static for the symbols.
                // This is safe because we keep the Library alive in the struct.
                let lib_ref: &'static Library = std::mem::transmute(&lib);

                Some(SoapySdrLib {
                    enumerate: lib_ref.get(b"SoapySDRDevice_enumerate\0").ok()?,
                    enumerate_free: lib_ref.get(b"SoapySDRKwargsList_clear\0").ok()?,
                    make: lib_ref.get(b"SoapySDRDevice_make\0").ok()?,
                    unmake: lib_ref.get(b"SoapySDRDevice_unmake\0").ok()?,
                    get_hardware_key: lib_ref.get(b"SoapySDRDevice_getHardwareKey\0").ok()?,
                    get_hardware_info: lib_ref.get(b"SoapySDRDevice_getHardwareInfo\0").ok()?,
                    get_num_channels: lib_ref.get(b"SoapySDRDevice_getNumChannels\0").ok()?,
                    set_frequency: lib_ref.get(b"SoapySDRDevice_setFrequency\0").ok()?,
                    get_frequency: lib_ref.get(b"SoapySDRDevice_getFrequency\0").ok()?,
                    get_frequency_range: lib_ref.get(b"SoapySDRDevice_getFrequencyRange\0").ok()?,
                    set_sample_rate: lib_ref.get(b"SoapySDRDevice_setSampleRate\0").ok()?,
                    get_sample_rate: lib_ref.get(b"SoapySDRDevice_getSampleRate\0").ok()?,
                    get_sample_rate_range: lib_ref.get(b"SoapySDRDevice_getSampleRateRange\0").ok()?,
                    set_bandwidth: lib_ref.get(b"SoapySDRDevice_setBandwidth\0").ok()?,
                    get_bandwidth: lib_ref.get(b"SoapySDRDevice_getBandwidth\0").ok()?,
                    set_gain: lib_ref.get(b"SoapySDRDevice_setGain\0").ok()?,
                    get_gain: lib_ref.get(b"SoapySDRDevice_getGain\0").ok()?,
                    get_gain_range: lib_ref.get(b"SoapySDRDevice_getGainRange\0").ok()?,
                    set_antenna: lib_ref.get(b"SoapySDRDevice_setAntenna\0").ok()?,
                    get_antenna: lib_ref.get(b"SoapySDRDevice_getAntenna\0").ok()?,
                    list_antennas: lib_ref.get(b"SoapySDRDevice_listAntennas\0").ok()?,
                    setup_stream: lib_ref.get(b"SoapySDRDevice_setupStream\0").ok()?,
                    close_stream: lib_ref.get(b"SoapySDRDevice_closeStream\0").ok()?,
                    get_stream_mtu: lib_ref.get(b"SoapySDRDevice_getStreamMTU\0").ok()?,
                    activate_stream: lib_ref.get(b"SoapySDRDevice_activateStream\0").ok()?,
                    deactivate_stream: lib_ref.get(b"SoapySDRDevice_deactivateStream\0").ok()?,
                    read_stream: lib_ref.get(b"SoapySDRDevice_readStream\0").ok()?,
                    write_stream: lib_ref.get(b"SoapySDRDevice_writeStream\0").ok()?,
                    strings_free: lib_ref.get(b"SoapySDRStrings_clear\0").ok()?,
                    kwargs_clear: lib_ref.get(b"SoapySDRKwargs_clear\0").ok()?,
                    kwargs_set: lib_ref.get(b"SoapySDRKwargs_set\0").ok()?,
                    last_error: lib_ref.get(b"SoapySDR_errToStr\0").ok()?,
                    _lib: lib,
                })
            };

            if result.is_some() {
                tracing::info!("Loaded SoapySDR library: {}", name);
                return result;
            }
        }
    }
    tracing::debug!("SoapySDR library not found");
    None
}

/// Get the loaded library, initializing if necessary.
fn get_lib() -> Option<&'static SoapySdrLib> {
    SOAPYSDR_LIB.get_or_init(load_library).as_ref()
}

/// Check if libSoapySDR is available.
pub fn is_available() -> bool {
    get_lib().is_some()
}

/// Get the last error message.
pub fn last_error() -> String {
    get_lib()
        .map(|lib| {
            let ptr = unsafe { (lib.last_error)() };
            if ptr.is_null() {
                String::new()
            } else {
                unsafe { CStr::from_ptr(ptr).to_string_lossy().into_owned() }
            }
        })
        .unwrap_or_default()
}

/// Device information from enumeration.
#[derive(Debug, Clone)]
pub struct SoapyDeviceInfo {
    pub driver: String,
    pub label: String,
    pub serial: String,
    pub args: HashMap<String, String>,
}

/// Enumerate available SoapySDR devices.
pub fn enumerate_devices(filter: Option<&HashMap<String, String>>) -> Vec<SoapyDeviceInfo> {
    let lib = match get_lib() {
        Some(lib) => lib,
        None => return Vec::new(),
    };

    // Create filter kwargs if provided
    let mut filter_kwargs = SoapySDRKwargs {
        size: 0,
        keys: ptr::null_mut(),
        vals: ptr::null_mut(),
    };

    let _filter_cstrings: Vec<(CString, CString)>;
    if let Some(filter_map) = filter {
        _filter_cstrings = filter_map
            .iter()
            .filter_map(|(k, v)| {
                Some((CString::new(k.as_str()).ok()?, CString::new(v.as_str()).ok()?))
            })
            .collect();
        for (k, v) in &_filter_cstrings {
            unsafe { (lib.kwargs_set)(&mut filter_kwargs, k.as_ptr(), v.as_ptr()) };
        }
    }

    // Enumerate devices
    let results = unsafe { (lib.enumerate)(&filter_kwargs) };
    if results.is_null() {
        if filter_kwargs.size > 0 {
            unsafe { (lib.kwargs_clear)(&mut filter_kwargs) };
        }
        return Vec::new();
    }

    // Count results (they're null-terminated in the list... actually no, need to check size)
    // SoapySDR returns a NULL-terminated array
    let mut devices = Vec::new();
    let mut i = 0;
    loop {
        let kwargs_ptr = unsafe { results.add(i) };
        let kwargs = unsafe { &*kwargs_ptr };

        // Check if this is the end (size == 0 and keys == null)
        if kwargs.size == 0 && kwargs.keys.is_null() {
            break;
        }

        // Parse kwargs into device info
        let mut args = HashMap::new();
        for j in 0..kwargs.size {
            let key_ptr = unsafe { *kwargs.keys.add(j) };
            let val_ptr = unsafe { *kwargs.vals.add(j) };
            if !key_ptr.is_null() && !val_ptr.is_null() {
                let key = unsafe { CStr::from_ptr(key_ptr).to_string_lossy().into_owned() };
                let val = unsafe { CStr::from_ptr(val_ptr).to_string_lossy().into_owned() };
                args.insert(key, val);
            }
        }

        let driver = args.get("driver").cloned().unwrap_or_default();
        let label = args.get("label").cloned().unwrap_or_else(|| {
            args.get("product").cloned().unwrap_or_else(|| driver.clone())
        });
        let serial = args.get("serial").cloned().unwrap_or_default();

        devices.push(SoapyDeviceInfo {
            driver,
            label,
            serial,
            args,
        });

        i += 1;
        if i > 100 {
            // Safety limit
            break;
        }
    }

    // Free the enumeration results
    unsafe { (lib.enumerate_free)(results, devices.len()) };

    // Clean up filter kwargs
    if filter_kwargs.size > 0 {
        unsafe { (lib.kwargs_clear)(&mut filter_kwargs) };
    }

    devices
}

/// Safe wrapper around a SoapySDR device.
pub struct SoapyDevice {
    handle: SoapyDeviceHandle,
    info: SoapyDeviceInfo,
    rx_channels: usize,
    tx_channels: usize,
}

// SAFETY: SoapySDR devices are designed to be thread-safe.
unsafe impl Send for SoapyDevice {}

impl SoapyDevice {
    /// Create a device from enumeration info.
    pub fn make(info: &SoapyDeviceInfo) -> SoapySdrResult<Self> {
        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;

        // Build kwargs from info.args
        let mut kwargs = SoapySDRKwargs {
            size: 0,
            keys: ptr::null_mut(),
            vals: ptr::null_mut(),
        };

        let cstrings: Vec<(CString, CString)> = info
            .args
            .iter()
            .filter_map(|(k, v)| {
                Some((CString::new(k.as_str()).ok()?, CString::new(v.as_str()).ok()?))
            })
            .collect();

        for (k, v) in &cstrings {
            unsafe { (lib.kwargs_set)(&mut kwargs, k.as_ptr(), v.as_ptr()) };
        }

        let handle = unsafe { (lib.make)(&kwargs) };

        // Clean up kwargs
        if kwargs.size > 0 {
            unsafe { (lib.kwargs_clear)(&mut kwargs) };
        }

        if handle.is_null() {
            return Err(SoapySdrError::CreateFailed(last_error()));
        }

        // Get channel counts
        let rx_channels = unsafe { (lib.get_num_channels)(handle, SOAPY_SDR_RX) };
        let tx_channels = unsafe { (lib.get_num_channels)(handle, SOAPY_SDR_TX) };

        tracing::info!(
            "Opened SoapySDR device: {} ({} RX, {} TX channels)",
            info.label,
            rx_channels,
            tx_channels
        );

        Ok(Self {
            handle,
            info: info.clone(),
            rx_channels,
            tx_channels,
        })
    }

    /// Create a device from a connection string (key=value,key=value).
    pub fn make_from_string(args: &str) -> SoapySdrResult<Self> {
        let mut map = HashMap::new();
        for pair in args.split(',') {
            if let Some(pos) = pair.find('=') {
                let key = pair[..pos].trim().to_string();
                let value = pair[pos + 1..].trim().to_string();
                map.insert(key, value);
            }
        }

        let info = SoapyDeviceInfo {
            driver: map.get("driver").cloned().unwrap_or_default(),
            label: map.get("label").cloned().unwrap_or_else(|| "SoapySDR".to_string()),
            serial: map.get("serial").cloned().unwrap_or_default(),
            args: map,
        };

        Self::make(&info)
    }

    /// Get device info.
    pub fn info(&self) -> &SoapyDeviceInfo {
        &self.info
    }

    /// Get hardware key (driver type).
    pub fn hardware_key(&self) -> String {
        let lib = match get_lib() {
            Some(lib) => lib,
            None => return String::new(),
        };
        let ptr = unsafe { (lib.get_hardware_key)(self.handle) };
        if ptr.is_null() {
            String::new()
        } else {
            unsafe { CStr::from_ptr(ptr).to_string_lossy().into_owned() }
        }
    }

    /// Get number of RX channels.
    pub fn num_rx_channels(&self) -> usize {
        self.rx_channels
    }

    /// Get number of TX channels.
    pub fn num_tx_channels(&self) -> usize {
        self.tx_channels
    }

    /// Set center frequency in Hz.
    pub fn set_frequency(&self, direction: c_int, channel: usize, freq_hz: f64) -> SoapySdrResult<()> {
        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;
        let ret = unsafe {
            (lib.set_frequency)(self.handle, direction, channel, freq_hz, ptr::null())
        };
        if ret != 0 {
            Err(SoapySdrError::OperationFailed(format!(
                "set_frequency: {}",
                last_error()
            )))
        } else {
            Ok(())
        }
    }

    /// Get center frequency in Hz.
    pub fn get_frequency(&self, direction: c_int, channel: usize) -> f64 {
        get_lib()
            .map(|lib| unsafe { (lib.get_frequency)(self.handle, direction, channel) })
            .unwrap_or(0.0)
    }

    /// Get frequency range.
    pub fn get_frequency_range(&self, direction: c_int, channel: usize) -> (f64, f64) {
        let lib = match get_lib() {
            Some(lib) => lib,
            None => return (0.0, 0.0),
        };
        let mut length: usize = 0;
        let ranges = unsafe { (lib.get_frequency_range)(self.handle, direction, channel, &mut length) };
        if ranges.is_null() || length == 0 {
            return (0.0, 0.0);
        }
        // Take the first range (usually the main tuning range)
        let range = unsafe { *ranges };
        (range.minimum, range.maximum)
    }

    /// Set sample rate in Hz.
    pub fn set_sample_rate(&self, direction: c_int, channel: usize, rate: f64) -> SoapySdrResult<()> {
        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.set_sample_rate)(self.handle, direction, channel, rate) };
        if ret != 0 {
            Err(SoapySdrError::OperationFailed(format!(
                "set_sample_rate: {}",
                last_error()
            )))
        } else {
            Ok(())
        }
    }

    /// Get sample rate in Hz.
    pub fn get_sample_rate(&self, direction: c_int, channel: usize) -> f64 {
        get_lib()
            .map(|lib| unsafe { (lib.get_sample_rate)(self.handle, direction, channel) })
            .unwrap_or(0.0)
    }

    /// Get sample rate range.
    pub fn get_sample_rate_range(&self, direction: c_int, channel: usize) -> (f64, f64) {
        let lib = match get_lib() {
            Some(lib) => lib,
            None => return (0.0, 0.0),
        };
        let mut length: usize = 0;
        let ranges = unsafe { (lib.get_sample_rate_range)(self.handle, direction, channel, &mut length) };
        if ranges.is_null() || length == 0 {
            return (0.0, 0.0);
        }
        // Return overall min/max from all ranges
        let mut min = f64::MAX;
        let mut max = f64::MIN;
        for i in 0..length {
            let range = unsafe { *ranges.add(i) };
            if range.minimum < min {
                min = range.minimum;
            }
            if range.maximum > max {
                max = range.maximum;
            }
        }
        (min, max)
    }

    /// Set bandwidth in Hz.
    pub fn set_bandwidth(&self, direction: c_int, channel: usize, bw: f64) -> SoapySdrResult<()> {
        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.set_bandwidth)(self.handle, direction, channel, bw) };
        if ret != 0 {
            Err(SoapySdrError::OperationFailed(format!(
                "set_bandwidth: {}",
                last_error()
            )))
        } else {
            Ok(())
        }
    }

    /// Get bandwidth in Hz.
    pub fn get_bandwidth(&self, direction: c_int, channel: usize) -> f64 {
        get_lib()
            .map(|lib| unsafe { (lib.get_bandwidth)(self.handle, direction, channel) })
            .unwrap_or(0.0)
    }

    /// Set gain in dB.
    pub fn set_gain(&self, direction: c_int, channel: usize, gain: f64) -> SoapySdrResult<()> {
        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;
        let ret = unsafe { (lib.set_gain)(self.handle, direction, channel, gain) };
        if ret != 0 {
            Err(SoapySdrError::OperationFailed(format!(
                "set_gain: {}",
                last_error()
            )))
        } else {
            Ok(())
        }
    }

    /// Get gain in dB.
    pub fn get_gain(&self, direction: c_int, channel: usize) -> f64 {
        get_lib()
            .map(|lib| unsafe { (lib.get_gain)(self.handle, direction, channel) })
            .unwrap_or(0.0)
    }

    /// Get gain range in dB.
    pub fn get_gain_range(&self, direction: c_int, channel: usize) -> (f64, f64) {
        get_lib()
            .map(|lib| {
                let range = unsafe { (lib.get_gain_range)(self.handle, direction, channel) };
                (range.minimum, range.maximum)
            })
            .unwrap_or((0.0, 0.0))
    }

    /// Set antenna.
    pub fn set_antenna(&self, direction: c_int, channel: usize, antenna: &str) -> SoapySdrResult<()> {
        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;
        let antenna_cstr = CString::new(antenna).map_err(|_| {
            SoapySdrError::InvalidConfig("Invalid antenna name".to_string())
        })?;
        let ret = unsafe { (lib.set_antenna)(self.handle, direction, channel, antenna_cstr.as_ptr()) };
        if ret != 0 {
            Err(SoapySdrError::OperationFailed(format!(
                "set_antenna: {}",
                last_error()
            )))
        } else {
            Ok(())
        }
    }

    /// Get current antenna.
    pub fn get_antenna(&self, direction: c_int, channel: usize) -> String {
        let lib = match get_lib() {
            Some(lib) => lib,
            None => return String::new(),
        };
        let ptr = unsafe { (lib.get_antenna)(self.handle, direction, channel) };
        if ptr.is_null() {
            String::new()
        } else {
            unsafe { CStr::from_ptr(ptr).to_string_lossy().into_owned() }
        }
    }

    /// List available antennas.
    pub fn list_antennas(&self, direction: c_int, channel: usize) -> Vec<String> {
        let lib = match get_lib() {
            Some(lib) => lib,
            None => return Vec::new(),
        };
        let mut length: usize = 0;
        let list = unsafe { (lib.list_antennas)(self.handle, direction, channel, &mut length) };
        if list.is_null() || length == 0 {
            return Vec::new();
        }

        let mut antennas = Vec::with_capacity(length);
        for i in 0..length {
            let ptr = unsafe { *list.add(i) };
            if !ptr.is_null() {
                antennas.push(unsafe { CStr::from_ptr(ptr).to_string_lossy().into_owned() });
            }
        }

        // Free the string list
        unsafe { (lib.strings_free)(list, length) };

        antennas
    }

    /// Setup a stream.
    pub fn setup_stream(
        &self,
        direction: c_int,
        channels: &[usize],
    ) -> SoapySdrResult<SoapyStream> {
        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;

        let format = SOAPY_SDR_CF32.as_ptr() as *const c_char;
        let channel_vec: Vec<usize> = channels.iter().map(|&c| c).collect();

        let stream = unsafe {
            (lib.setup_stream)(
                self.handle,
                direction,
                format,
                channel_vec.as_ptr(),
                channel_vec.len(),
                ptr::null(),
            )
        };

        if stream.is_null() {
            return Err(SoapySdrError::StreamSetupFailed(last_error()));
        }

        let mtu = unsafe { (lib.get_stream_mtu)(self.handle, stream) };

        Ok(SoapyStream {
            device_handle: self.handle,
            stream_handle: stream,
            direction,
            mtu,
            active: false,
        })
    }
}

impl Drop for SoapyDevice {
    fn drop(&mut self) {
        if let Some(lib) = get_lib() {
            tracing::debug!("Closing SoapySDR device: {}", self.info.label);
            unsafe { (lib.unmake)(self.handle) };
        }
    }
}

/// SoapySDR stream wrapper.
pub struct SoapyStream {
    device_handle: SoapyDeviceHandle,
    stream_handle: SoapyStreamHandle,
    direction: c_int,
    mtu: usize,
    active: bool,
}

// SAFETY: Stream operations are synchronized internally.
unsafe impl Send for SoapyStream {}

impl SoapyStream {
    /// Get the stream MTU (maximum samples per read/write).
    pub fn mtu(&self) -> usize {
        self.mtu
    }

    /// Get the stream direction.
    pub fn direction(&self) -> c_int {
        self.direction
    }

    /// Check if the stream is active.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Activate the stream.
    pub fn activate(&mut self) -> SoapySdrResult<()> {
        if self.active {
            return Ok(());
        }

        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;
        let ret = unsafe {
            (lib.activate_stream)(self.device_handle, self.stream_handle, 0, 0, 0)
        };

        if ret != 0 {
            Err(SoapySdrError::OperationFailed(format!(
                "activate_stream: {}",
                last_error()
            )))
        } else {
            self.active = true;
            Ok(())
        }
    }

    /// Deactivate the stream.
    pub fn deactivate(&mut self) -> SoapySdrResult<()> {
        if !self.active {
            return Ok(());
        }

        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;
        let ret = unsafe {
            (lib.deactivate_stream)(self.device_handle, self.stream_handle, 0, 0)
        };

        if ret != 0 {
            Err(SoapySdrError::OperationFailed(format!(
                "deactivate_stream: {}",
                last_error()
            )))
        } else {
            self.active = false;
            Ok(())
        }
    }

    /// Read samples from an RX stream.
    ///
    /// Returns (samples_read, flags, timestamp_ns).
    pub fn read(
        &self,
        buffer: &mut [[f32; 2]],
        timeout_us: i64,
    ) -> SoapySdrResult<(usize, i32, i64)> {
        if !self.active {
            return Err(SoapySdrError::OperationFailed("Stream not active".to_string()));
        }

        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;

        let mut flags: c_int = 0;
        let mut time_ns: i64 = 0;
        let mut buf_ptr = buffer.as_mut_ptr() as *mut c_void;

        let ret = unsafe {
            (lib.read_stream)(
                self.device_handle,
                self.stream_handle,
                &mut buf_ptr,
                buffer.len(),
                &mut flags,
                &mut time_ns,
                timeout_us,
            )
        };

        if ret < 0 {
            Err(SoapySdrError::ReadFailed(format!(
                "read_stream returned {}: {}",
                ret,
                last_error()
            )))
        } else {
            Ok((ret as usize, flags, time_ns))
        }
    }

    /// Write samples to a TX stream.
    ///
    /// Returns samples_written.
    pub fn write(
        &self,
        buffer: &[[f32; 2]],
        flags: i32,
        time_ns: i64,
        timeout_us: i64,
    ) -> SoapySdrResult<usize> {
        if !self.active {
            return Err(SoapySdrError::OperationFailed("Stream not active".to_string()));
        }

        let lib = get_lib().ok_or(SoapySdrError::LibraryNotFound)?;

        let mut flags_out: c_int = flags;
        let buf_ptr = buffer.as_ptr() as *const c_void;

        let ret = unsafe {
            (lib.write_stream)(
                self.device_handle,
                self.stream_handle,
                &buf_ptr,
                buffer.len(),
                &mut flags_out,
                time_ns,
                timeout_us,
            )
        };

        if ret < 0 {
            Err(SoapySdrError::WriteFailed(format!(
                "write_stream returned {}: {}",
                ret,
                last_error()
            )))
        } else {
            Ok(ret as usize)
        }
    }

    /// Close the stream.
    fn close(&mut self) {
        if let Some(lib) = get_lib() {
            if self.active {
                let _ = self.deactivate();
            }
            unsafe { (lib.close_stream)(self.device_handle, self.stream_handle) };
        }
    }
}

impl Drop for SoapyStream {
    fn drop(&mut self) {
        self.close();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_library_availability() {
        let available = is_available();
        if available {
            println!("libSoapySDR is available");
        } else {
            println!("libSoapySDR not available (expected on most dev machines)");
        }
    }

    #[test]
    fn test_enumerate_devices() {
        if !is_available() {
            println!("Skipping test - libSoapySDR not available");
            return;
        }

        let devices = enumerate_devices(None);
        println!("Found {} SoapySDR devices:", devices.len());
        for dev in &devices {
            println!("  - {} (driver: {}, serial: {})", dev.label, dev.driver, dev.serial);
        }
    }
}
