//! FPGA types and data structures

use std::collections::HashMap;
use num_complex::Complex64;

/// I/Q sample type (compatible with r4w-core::IQSample)
pub type IQSample = Complex64;

/// FPGA platform identifier
#[derive(Debug, Clone, PartialEq)]
pub enum FpgaPlatform {
    /// Xilinx Zynq-7000 series (ARM Cortex-A9 + FPGA)
    XilinxZynq7000 { part: String },

    /// Xilinx Zynq UltraScale+ (ARM Cortex-A53 + FPGA)
    XilinxZynqUltraScale { part: String },

    /// Lattice iCE40 series (low power, open toolchain)
    LatticeIce40 { variant: String },

    /// Lattice ECP5 series (mid-range, open toolchain)
    LatticeEcp5 { variant: String },

    /// Intel/Altera Cyclone series
    IntelCyclone { variant: String },

    /// Software simulation (no hardware)
    Simulated,

    /// Other/unknown platform
    Other(String),
}

impl FpgaPlatform {
    /// Get a human-readable name
    pub fn name(&self) -> &str {
        match self {
            FpgaPlatform::XilinxZynq7000 { .. } => "Xilinx Zynq-7000",
            FpgaPlatform::XilinxZynqUltraScale { .. } => "Xilinx Zynq UltraScale+",
            FpgaPlatform::LatticeIce40 { .. } => "Lattice iCE40",
            FpgaPlatform::LatticeEcp5 { .. } => "Lattice ECP5",
            FpgaPlatform::IntelCyclone { .. } => "Intel Cyclone",
            FpgaPlatform::Simulated => "Simulated",
            FpgaPlatform::Other(name) => name,
        }
    }

    /// Check if this platform supports open-source toolchains
    pub fn has_open_toolchain(&self) -> bool {
        matches!(
            self,
            FpgaPlatform::LatticeIce40 { .. }
                | FpgaPlatform::LatticeEcp5 { .. }
                | FpgaPlatform::Simulated
        )
    }
}

/// Information about the FPGA device
#[derive(Debug, Clone)]
pub struct FpgaInfo {
    /// Platform type
    pub platform: FpgaPlatform,

    /// Device identifier string
    pub device: String,

    /// Loaded bitstream version (if known)
    pub bitstream_version: Option<String>,

    /// Bitstream name/identifier
    pub bitstream_name: Option<String>,

    /// Build timestamp of bitstream
    pub bitstream_timestamp: Option<String>,

    /// Driver version
    pub driver_version: Option<String>,
}

/// FPGA capabilities
#[derive(Debug, Clone)]
pub struct FpgaCapabilities {
    /// Maximum FFT size supported
    pub max_fft_size: usize,

    /// Maximum FIR filter taps
    pub max_fir_taps: usize,

    /// List of supported waveform accelerators
    pub supported_waveforms: Vec<String>,

    /// DMA buffer size in bytes
    pub dma_buffer_size: usize,

    /// FPGA fabric clock frequency in Hz
    pub clock_frequency_hz: u64,

    /// Number of DSP blocks available
    pub dsp_blocks: usize,

    /// Number of logic cells/LUTs
    pub logic_cells: usize,

    /// Block RAM size in bytes
    pub bram_bytes: usize,

    /// Available IP cores
    pub ip_cores: Vec<IpCore>,

    /// Number of DMA channels
    pub dma_channels: usize,

    /// Supports streaming mode
    pub supports_streaming: bool,

    /// Supports interrupt-driven I/O
    pub supports_interrupts: bool,
}

impl Default for FpgaCapabilities {
    fn default() -> Self {
        Self {
            max_fft_size: 1024,
            max_fir_taps: 64,
            supported_waveforms: vec![],
            dma_buffer_size: 64 * 1024,
            clock_frequency_hz: 100_000_000,
            dsp_blocks: 0,
            logic_cells: 0,
            bram_bytes: 0,
            ip_cores: vec![],
            dma_channels: 1,
            supports_streaming: false,
            supports_interrupts: false,
        }
    }
}

/// IP core information
#[derive(Debug, Clone)]
pub struct IpCore {
    /// Core name (e.g., "r4w_fft_1024")
    pub name: String,

    /// Core type
    pub core_type: IpCoreType,

    /// Version string
    pub version: String,

    /// Base address in memory map
    pub base_address: usize,

    /// Address range size
    pub address_size: usize,

    /// Whether the core is currently available
    pub available: bool,
}

/// Types of IP cores
#[derive(Debug, Clone, PartialEq)]
pub enum IpCoreType {
    /// FFT/IFFT processor
    Fft { size: usize },

    /// FIR filter
    Fir { max_taps: usize },

    /// Chirp generator (for LoRa)
    ChirpGenerator,

    /// Chirp correlator (for LoRa demodulation)
    ChirpCorrelator,

    /// NCO/DDS carrier generator
    Nco,

    /// CORDIC for sin/cos
    Cordic,

    /// Symbol detector (matched filter)
    SymbolDetector,

    /// DMA controller
    DmaController,

    /// Generic AXI peripheral
    AxiPeripheral { name: String },

    /// Custom/unknown core type
    Custom(String),
}

/// Register map for an IP core
#[derive(Debug, Clone)]
pub struct RegisterMap {
    /// Base address
    pub base: usize,

    /// Named registers with their offsets
    pub registers: HashMap<String, RegisterInfo>,
}

/// Information about a single register
#[derive(Debug, Clone)]
pub struct RegisterInfo {
    /// Offset from base address
    pub offset: usize,

    /// Width in bits
    pub width: u8,

    /// Access mode
    pub access: RegisterAccess,

    /// Description
    pub description: String,
}

/// Register access mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RegisterAccess {
    /// Read-only
    ReadOnly,
    /// Write-only
    WriteOnly,
    /// Read-write
    ReadWrite,
    /// Write-once (self-clearing)
    WriteOnce,
}

/// DMA buffer for transferring samples
#[derive(Debug)]
pub struct DmaBuffer {
    /// Physical address (for FPGA access)
    pub phys_addr: usize,

    /// Virtual address (for CPU access)
    pub virt_addr: *mut u8,

    /// Buffer size in bytes
    pub size: usize,

    /// Whether this buffer is currently in use
    pub in_use: bool,
}

// Safety: DmaBuffer is Send if properly managed
unsafe impl Send for DmaBuffer {}

/// Streaming configuration
#[derive(Debug, Clone)]
pub struct StreamConfig {
    /// Sample rate in Hz
    pub sample_rate: f64,

    /// Buffer size in samples
    pub buffer_size: usize,

    /// Number of buffers for double/triple buffering
    pub num_buffers: usize,

    /// Whether to enable continuous streaming
    pub continuous: bool,

    /// Timeout for buffer operations in milliseconds
    pub timeout_ms: u32,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            sample_rate: 1_000_000.0,
            buffer_size: 4096,
            num_buffers: 2,
            continuous: true,
            timeout_ms: 1000,
        }
    }
}

/// Handle to an active stream
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct StreamHandle(pub u32);

/// Statistics for a running stream
#[derive(Debug, Clone, Default)]
pub struct StreamStats {
    /// Total samples transferred
    pub samples_transferred: u64,

    /// Number of buffer overruns
    pub overruns: u64,

    /// Number of buffer underruns
    pub underruns: u64,

    /// Current throughput in samples/sec
    pub throughput_sps: f64,

    /// Average latency in microseconds
    pub avg_latency_us: f64,

    /// Maximum latency in microseconds
    pub max_latency_us: f64,
}
