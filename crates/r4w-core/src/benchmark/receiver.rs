//! UDP Sample Receiver for Benchmarking
//!
//! Receives I/Q samples over UDP in various formats.
//! Uses the unified `IqFormat` for actual parsing.

use crate::io::IqFormat;
use crate::types::IQSample;
use std::io;
use std::net::UdpSocket;
use std::time::Duration;

/// Sample format for UDP data.
///
/// This is a simplified format enum for the benchmark module.
/// It wraps the unified `IqFormat` for actual I/O operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SampleFormat {
    /// 32-bit float I/Q pairs (8 bytes per sample)
    #[default]
    Float32,
    /// 16-bit signed integer I/Q pairs (4 bytes per sample)
    Int16,
}

impl SampleFormat {
    /// Bytes per I/Q sample
    pub fn bytes_per_sample(&self) -> usize {
        self.to_iq_format().bytes_per_sample()
    }

    /// Get format name
    pub fn name(&self) -> &'static str {
        match self {
            SampleFormat::Float32 => "f32",
            SampleFormat::Int16 => "i16",
        }
    }

    /// Parse format from string
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "f32" | "float32" | "float" | "cf32" | "ettus" => Some(SampleFormat::Float32),
            "i16" | "int16" | "short" | "ci16" | "sc16" => Some(SampleFormat::Int16),
            _ => None,
        }
    }

    /// Convert to unified IqFormat
    pub fn to_iq_format(&self) -> IqFormat {
        match self {
            SampleFormat::Float32 => IqFormat::Cf32,
            SampleFormat::Int16 => IqFormat::Ci16,
        }
    }

    /// Create from IqFormat (returns None for unsupported formats)
    pub fn from_iq_format(fmt: IqFormat) -> Option<Self> {
        match fmt {
            IqFormat::Cf32 => Some(SampleFormat::Float32),
            IqFormat::Ci16 => Some(SampleFormat::Int16),
            _ => None, // Cf64, Ci8, Cu8 not supported in benchmark
        }
    }
}

impl std::fmt::Display for SampleFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

impl From<SampleFormat> for IqFormat {
    fn from(fmt: SampleFormat) -> Self {
        fmt.to_iq_format()
    }
}

/// UDP receiver for I/Q samples
pub struct BenchmarkReceiver {
    socket: UdpSocket,
    format: IqFormat,
    buffer: Vec<u8>,
    /// Total packets received
    pub packets_received: u64,
    /// Total samples received
    pub samples_received: u64,
    /// Total bytes received
    pub bytes_received: u64,
    /// Receive errors
    pub errors: u64,
}

impl BenchmarkReceiver {
    /// Create a new receiver bound to the specified port
    pub fn bind(port: u16, format: SampleFormat) -> io::Result<Self> {
        Self::bind_with_iq_format(port, format.to_iq_format())
    }

    /// Create a new receiver with explicit IqFormat
    pub fn bind_with_iq_format(port: u16, format: IqFormat) -> io::Result<Self> {
        let socket = UdpSocket::bind(format!("0.0.0.0:{}", port))?;
        // Set non-blocking for timeout support
        socket.set_nonblocking(false)?;

        Ok(Self {
            socket,
            format,
            buffer: vec![0u8; 65536], // Max UDP packet size
            packets_received: 0,
            samples_received: 0,
            bytes_received: 0,
            errors: 0,
        })
    }

    /// Set receive timeout
    pub fn set_timeout(&self, timeout: Option<Duration>) -> io::Result<()> {
        self.socket.set_read_timeout(timeout)
    }

    /// Get the local address
    pub fn local_addr(&self) -> io::Result<std::net::SocketAddr> {
        self.socket.local_addr()
    }

    /// Receive a batch of samples (blocking with timeout)
    pub fn recv_batch(&mut self, timeout: Duration) -> io::Result<Vec<IQSample>> {
        self.socket.set_read_timeout(Some(timeout))?;
        self.recv_internal()
    }

    /// Receive a batch of samples (blocking, no timeout)
    pub fn recv_blocking(&mut self) -> io::Result<Vec<IQSample>> {
        self.socket.set_read_timeout(None)?;
        self.recv_internal()
    }

    /// Try to receive samples (non-blocking)
    pub fn try_recv(&mut self) -> io::Result<Option<Vec<IQSample>>> {
        self.socket.set_nonblocking(true)?;
        let result = self.recv_internal();
        self.socket.set_nonblocking(false)?;

        match result {
            Ok(samples) => Ok(Some(samples)),
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(e),
        }
    }

    fn recv_internal(&mut self) -> io::Result<Vec<IQSample>> {
        match self.socket.recv(&mut self.buffer) {
            Ok(len) => {
                self.packets_received += 1;
                self.bytes_received += len as u64;

                // Use unified IqFormat for parsing
                let samples = self.format.parse_bytes(&self.buffer[..len]);
                self.samples_received += samples.len() as u64;
                Ok(samples)
            }
            Err(e) => {
                if e.kind() != io::ErrorKind::WouldBlock && e.kind() != io::ErrorKind::TimedOut {
                    self.errors += 1;
                }
                Err(e)
            }
        }
    }

    /// Reset statistics
    pub fn reset_stats(&mut self) {
        self.packets_received = 0;
        self.samples_received = 0;
        self.bytes_received = 0;
        self.errors = 0;
    }

    /// Get sample format (as legacy SampleFormat)
    pub fn format(&self) -> SampleFormat {
        SampleFormat::from_iq_format(self.format).unwrap_or(SampleFormat::Float32)
    }

    /// Get sample format as IqFormat
    pub fn iq_format(&self) -> IqFormat {
        self.format
    }
}

/// UDP sender for I/Q samples (for testing)
pub struct BenchmarkSender {
    socket: UdpSocket,
    format: IqFormat,
    target: std::net::SocketAddr,
}

impl BenchmarkSender {
    /// Create a new sender
    pub fn new(target: &str, format: SampleFormat) -> io::Result<Self> {
        Self::new_with_iq_format(target, format.to_iq_format())
    }

    /// Create a new sender with explicit IqFormat
    pub fn new_with_iq_format(target: &str, format: IqFormat) -> io::Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        let target: std::net::SocketAddr = target
            .parse()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;

        Ok(Self {
            socket,
            format,
            target,
        })
    }

    /// Send samples
    pub fn send(&self, samples: &[IQSample]) -> io::Result<usize> {
        // Use unified IqFormat for serialization
        let data = self.format.to_bytes(samples);
        self.socket.send_to(&data, self.target)
    }

    /// Get the format
    pub fn format(&self) -> SampleFormat {
        SampleFormat::from_iq_format(self.format).unwrap_or(SampleFormat::Float32)
    }

    /// Get sample format as IqFormat
    pub fn iq_format(&self) -> IqFormat {
        self.format
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sample_format_parse() {
        assert_eq!(SampleFormat::from_str("f32"), Some(SampleFormat::Float32));
        assert_eq!(SampleFormat::from_str("float32"), Some(SampleFormat::Float32));
        assert_eq!(SampleFormat::from_str("ettus"), Some(SampleFormat::Float32));
        assert_eq!(SampleFormat::from_str("i16"), Some(SampleFormat::Int16));
        assert_eq!(SampleFormat::from_str("int16"), Some(SampleFormat::Int16));
        assert_eq!(SampleFormat::from_str("sc16"), Some(SampleFormat::Int16));
        assert_eq!(SampleFormat::from_str("invalid"), None);
    }

    #[test]
    fn test_sample_format_bytes() {
        assert_eq!(SampleFormat::Float32.bytes_per_sample(), 8);
        assert_eq!(SampleFormat::Int16.bytes_per_sample(), 4);
    }

    #[test]
    fn test_sample_format_to_iq_format() {
        assert_eq!(SampleFormat::Float32.to_iq_format(), IqFormat::Cf32);
        assert_eq!(SampleFormat::Int16.to_iq_format(), IqFormat::Ci16);
    }

    #[test]
    fn test_sample_format_from_iq_format() {
        assert_eq!(SampleFormat::from_iq_format(IqFormat::Cf32), Some(SampleFormat::Float32));
        assert_eq!(SampleFormat::from_iq_format(IqFormat::Ci16), Some(SampleFormat::Int16));
        assert_eq!(SampleFormat::from_iq_format(IqFormat::Cf64), None);
        assert_eq!(SampleFormat::from_iq_format(IqFormat::Cu8), None);
    }

    #[test]
    fn test_f32_roundtrip() {
        let samples = vec![
            IQSample::new(0.5, -0.5),
            IQSample::new(-1.0, 1.0),
        ];

        // Use IqFormat directly (same as what BenchmarkSender/Receiver use)
        let format = IqFormat::Cf32;
        let bytes = format.to_bytes(&samples);
        let parsed = format.parse_bytes(&bytes);

        assert_eq!(parsed.len(), samples.len());
        for (orig, dec) in samples.iter().zip(parsed.iter()) {
            assert!((orig.re - dec.re).abs() < 1e-6);
            assert!((orig.im - dec.im).abs() < 1e-6);
        }
    }

    #[test]
    fn test_i16_roundtrip() {
        let samples = vec![
            IQSample::new(0.5, -0.5),
            IQSample::new(-1.0, 1.0),
        ];

        let format = IqFormat::Ci16;
        let bytes = format.to_bytes(&samples);
        let parsed = format.parse_bytes(&bytes);

        assert_eq!(parsed.len(), samples.len());
        for (orig, dec) in samples.iter().zip(parsed.iter()) {
            // i16 precision is ~1/32768
            assert!((orig.re - dec.re).abs() < 1e-4);
            assert!((orig.im - dec.im).abs() < 1e-4);
        }
    }

    #[test]
    fn test_f32_parsing_backward_compat() {
        // Create test data: I=0.5, Q=-0.5
        let mut data = Vec::new();
        data.extend_from_slice(&0.5f32.to_le_bytes());
        data.extend_from_slice(&(-0.5f32).to_le_bytes());

        let samples = IqFormat::Cf32.parse_bytes(&data);
        assert_eq!(samples.len(), 1);
        assert!((samples[0].re - 0.5).abs() < 0.001);
        assert!((samples[0].im - (-0.5)).abs() < 0.001);
    }

    #[test]
    fn test_i16_parsing_backward_compat() {
        // Create test data: I=16384 (0.5), Q=-16384 (-0.5)
        let mut data = Vec::new();
        data.extend_from_slice(&16384i16.to_le_bytes());
        data.extend_from_slice(&(-16384i16).to_le_bytes());

        let samples = IqFormat::Ci16.parse_bytes(&data);
        assert_eq!(samples.len(), 1);
        assert!((samples[0].re - 0.5).abs() < 0.001);
        assert!((samples[0].im - (-0.5)).abs() < 0.001);
    }
}
