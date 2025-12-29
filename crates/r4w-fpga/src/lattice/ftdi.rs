//! FTDI device enumeration and basic interface
//!
//! This module provides discovery of FTDI USB devices which are commonly
//! used for programming and communicating with Lattice FPGAs.

use crate::error::{FpgaError, FpgaResult};

/// Information about an FTDI device
#[derive(Debug, Clone)]
pub struct FtdiDevice {
    /// Device index
    pub index: usize,

    /// USB vendor ID
    pub vendor_id: u16,

    /// USB product ID
    pub product_id: u16,

    /// Device description string
    pub description: String,

    /// Serial number
    pub serial: String,
}

/// Common FTDI product IDs
pub mod products {
    /// FT232R - Single channel
    pub const FT232R: u16 = 0x6001;
    /// FT2232H - Dual channel Hi-Speed
    pub const FT2232H: u16 = 0x6010;
    /// FT4232H - Quad channel Hi-Speed
    pub const FT4232H: u16 = 0x6011;
    /// FT232H - Single channel Hi-Speed
    pub const FT232H: u16 = 0x6014;
}

/// FTDI vendor ID
pub const FTDI_VID: u16 = 0x0403;

/// Enumerate available FTDI devices
pub fn enumerate_devices() -> FpgaResult<Vec<FtdiDevice>> {
    // This is a stub implementation.
    // Real implementation would use libftdi1 or similar.

    #[cfg(feature = "lattice")]
    {
        enumerate_devices_libftdi()
    }

    #[cfg(not(feature = "lattice"))]
    {
        // Return empty when lattice feature is disabled
        Ok(Vec::new())
    }
}

#[cfg(feature = "lattice")]
fn enumerate_devices_libftdi() -> FpgaResult<Vec<FtdiDevice>> {
    use libftdi1_sys as ftdi;
    use std::ffi::CStr;
    use std::ptr;

    let mut devices = Vec::new();

    unsafe {
        // Create FTDI context
        let ctx = ftdi::ftdi_new();
        if ctx.is_null() {
            return Err(FpgaError::DeviceNotFound(
                "Failed to create FTDI context".to_string(),
            ));
        }

        // Get device list
        let mut dev_list: *mut ftdi::ftdi_device_list = ptr::null_mut();
        let count = ftdi::ftdi_usb_find_all(ctx, &mut dev_list, FTDI_VID as i32, 0);

        if count < 0 {
            ftdi::ftdi_free(ctx);
            return Ok(devices); // No devices found
        }

        // Iterate through devices
        let mut current = dev_list;
        let mut index = 0;

        while !current.is_null() {
            let dev = (*current).dev;

            // Get device strings
            let mut manufacturer = [0i8; 256];
            let mut description = [0i8; 256];
            let mut serial = [0i8; 256];

            let ret = ftdi::ftdi_usb_get_strings(
                ctx,
                dev,
                manufacturer.as_mut_ptr(),
                256,
                description.as_mut_ptr(),
                256,
                serial.as_mut_ptr(),
                256,
            );

            if ret >= 0 {
                let desc_str = CStr::from_ptr(description.as_ptr())
                    .to_string_lossy()
                    .into_owned();
                let serial_str = CStr::from_ptr(serial.as_ptr())
                    .to_string_lossy()
                    .into_owned();

                devices.push(FtdiDevice {
                    index,
                    vendor_id: FTDI_VID,
                    product_id: 0, // Would need to read from USB descriptor
                    description: desc_str,
                    serial: serial_str,
                });
            }

            current = (*current).next;
            index += 1;
        }

        // Clean up
        ftdi::ftdi_list_free(&mut dev_list);
        ftdi::ftdi_free(ctx);
    }

    Ok(devices)
}

/// Check if an FTDI device appears to be connected to a Lattice FPGA
pub fn is_lattice_programmer(device: &FtdiDevice) -> bool {
    let desc = device.description.to_lowercase();

    // Known Lattice programming cables and dev boards
    desc.contains("ice")
        || desc.contains("lattice")
        || desc.contains("ecp5")
        || desc.contains("ulx3s")
        || desc.contains("orangecrab")
        || desc.contains("tinyfpga")
        || desc.contains("fomu")
}
