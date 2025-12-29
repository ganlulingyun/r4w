//! Register definitions for R4W IP cores

/// R4W FFT IP Core registers
pub mod fft {
    /// Control register
    pub const CTRL: usize = 0x00;
    /// Size configuration (log2)
    pub const SIZE: usize = 0x04;
    /// Status register
    pub const STATUS: usize = 0x08;
    /// Input data register
    pub const DATA_IN: usize = 0x10;
    /// Output data register
    pub const DATA_OUT: usize = 0x14;

    // Control bits
    pub const CTRL_START: u32 = 1 << 0;
    pub const CTRL_INVERSE: u32 = 1 << 1;
    pub const CTRL_RESET: u32 = 1 << 31;

    // Status bits
    pub const STATUS_DONE: u32 = 1 << 0;
    pub const STATUS_BUSY: u32 = 1 << 1;
    pub const STATUS_ERROR: u32 = 1 << 2;
}

/// R4W FIR IP Core registers
pub mod fir {
    /// Control register
    pub const CTRL: usize = 0x00;
    /// Number of taps
    pub const NUM_TAPS: usize = 0x04;
    /// Status register
    pub const STATUS: usize = 0x08;
    /// Input data register
    pub const DATA_IN: usize = 0x10;
    /// Output data register
    pub const DATA_OUT: usize = 0x14;
    /// Tap coefficient base (taps at 0x100, 0x104, ...)
    pub const TAPS_BASE: usize = 0x100;

    // Control bits
    pub const CTRL_START: u32 = 1 << 0;
    pub const CTRL_RELOAD_TAPS: u32 = 1 << 1;
    pub const CTRL_RESET: u32 = 1 << 31;

    // Status bits
    pub const STATUS_DONE: u32 = 1 << 0;
    pub const STATUS_BUSY: u32 = 1 << 1;
    pub const STATUS_ERROR: u32 = 1 << 2;
}

/// R4W Chirp Generator IP Core registers
pub mod chirp {
    /// Control register
    pub const CTRL: usize = 0x00;
    /// Spreading factor
    pub const SF: usize = 0x04;
    /// Status register
    pub const STATUS: usize = 0x08;
    /// Output data register
    pub const DATA_OUT: usize = 0x10;
    /// Bandwidth configuration
    pub const BANDWIDTH: usize = 0x14;

    // Control bits
    pub const CTRL_START: u32 = 1 << 0;
    pub const CTRL_UPCHIRP: u32 = 1 << 1;
    pub const CTRL_CONTINUOUS: u32 = 1 << 2;
    pub const CTRL_RESET: u32 = 1 << 31;

    // Status bits
    pub const STATUS_DONE: u32 = 1 << 0;
    pub const STATUS_BUSY: u32 = 1 << 1;
    pub const STATUS_OVERFLOW: u32 = 1 << 2;
}

/// R4W Chirp Correlator IP Core registers
pub mod correlator {
    /// Control register
    pub const CTRL: usize = 0x00;
    /// Spreading factor
    pub const SF: usize = 0x04;
    /// Status register
    pub const STATUS: usize = 0x08;
    /// Input data register
    pub const DATA_IN: usize = 0x10;
    /// Symbol result
    pub const SYMBOL: usize = 0x20;
    /// Correlation magnitude
    pub const MAGNITUDE: usize = 0x24;
    /// Correlation threshold
    pub const THRESHOLD: usize = 0x28;

    // Control bits
    pub const CTRL_START: u32 = 1 << 0;
    pub const CTRL_RESET: u32 = 1 << 31;

    // Status bits
    pub const STATUS_DONE: u32 = 1 << 0;
    pub const STATUS_BUSY: u32 = 1 << 1;
    pub const STATUS_DETECTED: u32 = 1 << 2;
}

/// R4W NCO (Numerically Controlled Oscillator) registers
pub mod nco {
    /// Control register
    pub const CTRL: usize = 0x00;
    /// Frequency word
    pub const FREQ: usize = 0x04;
    /// Phase offset
    pub const PHASE: usize = 0x08;
    /// Output amplitude
    pub const AMPLITUDE: usize = 0x0C;
    /// Status register
    pub const STATUS: usize = 0x10;
    /// I output
    pub const I_OUT: usize = 0x14;
    /// Q output
    pub const Q_OUT: usize = 0x18;

    // Control bits
    pub const CTRL_ENABLE: u32 = 1 << 0;
    pub const CTRL_RESET_PHASE: u32 = 1 << 1;
    pub const CTRL_RESET: u32 = 1 << 31;
}

/// Common version register layout
pub mod version {
    /// Get major version from version register
    pub fn major(version: u32) -> u8 {
        ((version >> 16) & 0xFF) as u8
    }

    /// Get minor version from version register
    pub fn minor(version: u32) -> u8 {
        ((version >> 8) & 0xFF) as u8
    }

    /// Get patch version from version register
    pub fn patch(version: u32) -> u8 {
        (version & 0xFF) as u8
    }

    /// Format version as string
    pub fn format(version: u32) -> String {
        format!("{}.{}.{}", major(version), minor(version), patch(version))
    }
}

/// ID register values for IP core identification
pub mod id {
    /// R4W FFT IP core ID
    pub const FFT: u32 = 0x52345746; // "R4WF"
    /// R4W FIR IP core ID
    pub const FIR: u32 = 0x52344649; // "R4FI"
    /// R4W Chirp Generator ID
    pub const CHIRP_GEN: u32 = 0x52344347; // "R4CG"
    /// R4W Chirp Correlator ID
    pub const CHIRP_CORR: u32 = 0x52344343; // "R4CC"
    /// R4W NCO ID
    pub const NCO: u32 = 0x5234494F; // "R4IO"
}
