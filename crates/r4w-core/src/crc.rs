//! CRC (Cyclic Redundancy Check) Engine
//!
//! Provides generic CRC computation with table-based lookup for common polynomials.
//!
//! ## Supported Standards
//!
//! - CRC-8 (polynomial 0x07)
//! - CRC-8/CCITT (polynomial 0x07, init 0xFF)
//! - CRC-16/CCITT (polynomial 0x1021)
//! - CRC-16/IBM (polynomial 0x8005)
//! - CRC-32 (polynomial 0x04C11DB7, ISO 3309 / ITU-T V.42)
//! - CRC-32C (Castagnoli, polynomial 0x1EDC6F41)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::crc::{Crc32, CrcComputer};
//!
//! let mut crc = Crc32::new();
//! crc.update(b"Hello, world!");
//! let checksum = crc.finalize();
//! assert_eq!(checksum, 0xEBE6C6E6);
//! ```

/// Trait for CRC computation.
pub trait CrcComputer {
    /// The output type of the CRC (u8, u16, u32, etc.)
    type Output: Copy + PartialEq + std::fmt::LowerHex;

    /// Update the CRC with additional data.
    fn update(&mut self, data: &[u8]);

    /// Finalize and return the CRC value.
    fn finalize(&self) -> Self::Output;

    /// Reset the CRC to its initial state.
    fn reset(&mut self);

    /// Compute CRC of an entire buffer in one call.
    fn compute(data: &[u8]) -> Self::Output
    where
        Self: Sized + Default,
    {
        let mut crc = Self::default();
        crc.update(data);
        crc.finalize()
    }

    /// Verify that data matches an expected CRC.
    fn verify(&mut self, data: &[u8], expected: Self::Output) -> bool
    where
        Self: Sized,
    {
        self.reset();
        self.update(data);
        self.finalize() == expected
    }
}

// ============================================================================
// CRC-8
// ============================================================================

/// CRC-8 with polynomial 0x07.
#[derive(Clone)]
pub struct Crc8 {
    table: [u8; 256],
    value: u8,
    init: u8,
}

impl Crc8 {
    /// Create a new CRC-8 with standard polynomial 0x07.
    pub fn new() -> Self {
        Self::with_poly(0x07, 0x00)
    }

    /// Create a CRC-8 with custom polynomial and initial value.
    pub fn with_poly(poly: u8, init: u8) -> Self {
        let mut table = [0u8; 256];
        for i in 0..256u16 {
            let mut crc = i as u8;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ poly;
                } else {
                    crc <<= 1;
                }
            }
            table[i as usize] = crc;
        }
        Self {
            table,
            value: init,
            init,
        }
    }
}

impl Default for Crc8 {
    fn default() -> Self {
        Self::new()
    }
}

impl CrcComputer for Crc8 {
    type Output = u8;

    fn update(&mut self, data: &[u8]) {
        for &byte in data {
            self.value = self.table[(self.value ^ byte) as usize];
        }
    }

    fn finalize(&self) -> u8 {
        self.value
    }

    fn reset(&mut self) {
        self.value = self.init;
    }
}

// ============================================================================
// CRC-16
// ============================================================================

/// CRC-16 with configurable polynomial.
#[derive(Clone)]
pub struct Crc16 {
    table: [u16; 256],
    value: u16,
    init: u16,
    xor_out: u16,
    reflect_in: bool,
    reflect_out: bool,
}

impl Crc16 {
    /// CRC-16/CCITT (polynomial 0x1021, init 0xFFFF).
    /// Used in X.25, V.41, HDLC, XMODEM, Bluetooth, SD cards.
    pub fn ccitt() -> Self {
        Self::new(0x1021, 0xFFFF, 0x0000, false, false)
    }

    /// CRC-16/IBM (polynomial 0x8005, init 0x0000, reflected).
    /// Used in USB, Modbus, ARC.
    pub fn ibm() -> Self {
        Self::new(0x8005, 0x0000, 0x0000, true, true)
    }

    /// CRC-16/XMODEM (polynomial 0x1021, init 0x0000).
    pub fn xmodem() -> Self {
        Self::new(0x1021, 0x0000, 0x0000, false, false)
    }

    /// Create a CRC-16 with custom parameters.
    pub fn new(poly: u16, init: u16, xor_out: u16, reflect_in: bool, reflect_out: bool) -> Self {
        let mut table = [0u16; 256];
        for i in 0..256u32 {
            let mut crc = (i as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ poly;
                } else {
                    crc <<= 1;
                }
            }
            table[i as usize] = crc;
        }
        Self {
            table,
            value: init,
            init,
            xor_out,
            reflect_in,
            reflect_out,
        }
    }
}

impl Default for Crc16 {
    fn default() -> Self {
        Self::ccitt()
    }
}

fn reflect_byte(b: u8) -> u8 {
    let mut reflected = 0u8;
    for i in 0..8 {
        if b & (1 << i) != 0 {
            reflected |= 1 << (7 - i);
        }
    }
    reflected
}

fn reflect_u16(val: u16) -> u16 {
    let mut reflected = 0u16;
    for i in 0..16 {
        if val & (1 << i) != 0 {
            reflected |= 1 << (15 - i);
        }
    }
    reflected
}

impl CrcComputer for Crc16 {
    type Output = u16;

    fn update(&mut self, data: &[u8]) {
        for &byte in data {
            let b = if self.reflect_in {
                reflect_byte(byte)
            } else {
                byte
            };
            let idx = ((self.value >> 8) ^ (b as u16)) as u8;
            self.value = (self.value << 8) ^ self.table[idx as usize];
        }
    }

    fn finalize(&self) -> u16 {
        let val = if self.reflect_out {
            reflect_u16(self.value)
        } else {
            self.value
        };
        val ^ self.xor_out
    }

    fn reset(&mut self) {
        self.value = self.init;
    }
}

// ============================================================================
// CRC-32
// ============================================================================

/// CRC-32 (ISO 3309 / ITU-T V.42).
///
/// Polynomial: 0x04C11DB7, reflected I/O, init 0xFFFFFFFF, xor_out 0xFFFFFFFF.
/// Used in Ethernet, PKZIP, PNG, MPEG-2.
#[derive(Clone)]
pub struct Crc32 {
    table: Box<[u32; 256]>,
    value: u32,
}

impl Crc32 {
    /// Create a new CRC-32 (standard Ethernet/PKZIP polynomial).
    pub fn new() -> Self {
        // Use reflected polynomial 0xEDB88320 for efficient byte-at-a-time computation
        let poly: u32 = 0xEDB88320;
        let mut table = Box::new([0u32; 256]);
        for i in 0..256u32 {
            let mut crc = i;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ poly;
                } else {
                    crc >>= 1;
                }
            }
            table[i as usize] = crc;
        }
        Self {
            table,
            value: 0xFFFFFFFF,
        }
    }
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

impl CrcComputer for Crc32 {
    type Output = u32;

    fn update(&mut self, data: &[u8]) {
        for &byte in data {
            let idx = ((self.value ^ (byte as u32)) & 0xFF) as usize;
            self.value = (self.value >> 8) ^ self.table[idx];
        }
    }

    fn finalize(&self) -> u32 {
        self.value ^ 0xFFFFFFFF
    }

    fn reset(&mut self) {
        self.value = 0xFFFFFFFF;
    }
}

/// CRC-32C (Castagnoli).
///
/// Polynomial: 0x1EDC6F41 (reflected: 0x82F63B78).
/// Used in iSCSI, SCTP, ext4, Btrfs. Hardware-accelerated on modern CPUs.
#[derive(Clone)]
pub struct Crc32c {
    table: Box<[u32; 256]>,
    value: u32,
}

impl Crc32c {
    /// Create a new CRC-32C.
    pub fn new() -> Self {
        let poly: u32 = 0x82F63B78; // Reflected Castagnoli
        let mut table = Box::new([0u32; 256]);
        for i in 0..256u32 {
            let mut crc = i;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ poly;
                } else {
                    crc >>= 1;
                }
            }
            table[i as usize] = crc;
        }
        Self {
            table,
            value: 0xFFFFFFFF,
        }
    }
}

impl Default for Crc32c {
    fn default() -> Self {
        Self::new()
    }
}

impl CrcComputer for Crc32c {
    type Output = u32;

    fn update(&mut self, data: &[u8]) {
        for &byte in data {
            let idx = ((self.value ^ (byte as u32)) & 0xFF) as usize;
            self.value = (self.value >> 8) ^ self.table[idx];
        }
    }

    fn finalize(&self) -> u32 {
        self.value ^ 0xFFFFFFFF
    }

    fn reset(&mut self) {
        self.value = 0xFFFFFFFF;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc32_known_values() {
        // "123456789" should produce 0xCBF43926
        let data = b"123456789";
        let checksum = Crc32::compute(data);
        assert_eq!(
            checksum, 0xCBF43926,
            "CRC-32 of '123456789' should be 0xCBF43926, got 0x{:08X}",
            checksum
        );
    }

    #[test]
    fn test_crc32_hello_world() {
        let checksum = Crc32::compute(b"Hello, world!");
        assert_eq!(checksum, 0xEBE6C6E6);
    }

    #[test]
    fn test_crc32_empty() {
        let checksum = Crc32::compute(b"");
        assert_eq!(checksum, 0x00000000);
    }

    #[test]
    fn test_crc32_incremental() {
        // Incremental should match one-shot
        let mut crc = Crc32::new();
        crc.update(b"Hello, ");
        crc.update(b"world!");
        let incremental = crc.finalize();

        let oneshot = Crc32::compute(b"Hello, world!");
        assert_eq!(incremental, oneshot);
    }

    #[test]
    fn test_crc32c_known_values() {
        // "123456789" should produce 0xE3069283
        let checksum = Crc32c::compute(b"123456789");
        assert_eq!(
            checksum, 0xE3069283,
            "CRC-32C of '123456789' should be 0xE3069283, got 0x{:08X}",
            checksum
        );
    }

    #[test]
    fn test_crc8_basic() {
        let mut crc = Crc8::new();
        crc.update(b"123456789");
        let checksum = crc.finalize();
        assert_eq!(checksum, 0xF4, "CRC-8 of '123456789' should be 0xF4");
    }

    #[test]
    fn test_crc16_ccitt() {
        let checksum = Crc16::compute(b"123456789");
        assert_eq!(
            checksum, 0x29B1,
            "CRC-16/CCITT of '123456789' should be 0x29B1, got 0x{:04X}",
            checksum
        );
    }

    #[test]
    fn test_crc_verify() {
        let data = b"test data";
        let checksum = Crc32::compute(data);
        let mut crc = Crc32::new();
        assert!(crc.verify(data, checksum));
        assert!(!crc.verify(data, checksum + 1));
    }

    #[test]
    fn test_crc_reset() {
        let mut crc = Crc32::new();
        crc.update(b"first");
        crc.reset();
        crc.update(b"123456789");
        assert_eq!(crc.finalize(), 0xCBF43926);
    }
}
