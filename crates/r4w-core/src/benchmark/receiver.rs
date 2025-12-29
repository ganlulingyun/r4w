//! UDP Sample Receiver for Benchmarking
//!
//! Receives I/Q samples over UDP in f32 or i16 format.

use crate::types::IQSample;
use std::io;
use std::net::UdpSocket;
use std::time::Duration;

/// Sample format for UDP data
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleFormat {
    /// 32-bit float I/Q pairs (8 bytes per sample)
    Float32,
    /// 16-bit signed integer I/Q pairs (4 bytes per sample)
    Int16,
}

impl SampleFormat {
    /// Bytes per I/Q sample
    pub fn bytes_per_sample(&self) -> usize {
        match self {
            SampleFormat::Float32 => 8,
            SampleFormat::Int16 => 4,
        }
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
            "f32" | "float32" | "float" => Some(SampleFormat::Float32),
            "i16" | "int16" | "short" => Some(SampleFormat::Int16),
            _ => None,
        }
    }
}

impl std::fmt::Display for SampleFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SampleFormat::Float32 => write!(f, "f32"),
            SampleFormat::Int16 => write!(f, "i16"),
        }
    }
}

/// UDP receiver for I/Q samples
pub struct BenchmarkReceiver {
    socket: UdpSocket,
    format: SampleFormat,
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

                let samples = self.parse_samples(&self.buffer[..len]);
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

    fn parse_samples(&self, data: &[u8]) -> Vec<IQSample> {
        match self.format {
            SampleFormat::Float32 => self.parse_f32(data),
            SampleFormat::Int16 => self.parse_i16(data),
        }
    }

    fn parse_f32(&self, data: &[u8]) -> Vec<IQSample> {
        let num_samples = data.len() / 8;
        let mut samples = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let offset = i * 8;
            if offset + 8 <= data.len() {
                let i_bytes: [u8; 4] = [
                    data[offset],
                    data[offset + 1],
                    data[offset + 2],
                    data[offset + 3],
                ];
                let q_bytes: [u8; 4] = [
                    data[offset + 4],
                    data[offset + 5],
                    data[offset + 6],
                    data[offset + 7],
                ];

                let i_val = f32::from_le_bytes(i_bytes) as f64;
                let q_val = f32::from_le_bytes(q_bytes) as f64;
                samples.push(IQSample::new(i_val, q_val));
            }
        }

        samples
    }

    fn parse_i16(&self, data: &[u8]) -> Vec<IQSample> {
        let num_samples = data.len() / 4;
        let mut samples = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let offset = i * 4;
            if offset + 4 <= data.len() {
                let i_raw = i16::from_le_bytes([data[offset], data[offset + 1]]);
                let q_raw = i16::from_le_bytes([data[offset + 2], data[offset + 3]]);

                // Scale to [-1.0, 1.0]
                let i_val = i_raw as f64 / 32768.0;
                let q_val = q_raw as f64 / 32768.0;
                samples.push(IQSample::new(i_val, q_val));
            }
        }

        samples
    }

    /// Reset statistics
    pub fn reset_stats(&mut self) {
        self.packets_received = 0;
        self.samples_received = 0;
        self.bytes_received = 0;
        self.errors = 0;
    }

    /// Get sample format
    pub fn format(&self) -> SampleFormat {
        self.format
    }
}

/// UDP sender for I/Q samples (for testing)
pub struct BenchmarkSender {
    socket: UdpSocket,
    format: SampleFormat,
    target: std::net::SocketAddr,
}

impl BenchmarkSender {
    /// Create a new sender
    pub fn new(target: &str, format: SampleFormat) -> io::Result<Self> {
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
        let data = self.serialize_samples(samples);
        self.socket.send_to(&data, self.target)
    }

    fn serialize_samples(&self, samples: &[IQSample]) -> Vec<u8> {
        match self.format {
            SampleFormat::Float32 => self.serialize_f32(samples),
            SampleFormat::Int16 => self.serialize_i16(samples),
        }
    }

    fn serialize_f32(&self, samples: &[IQSample]) -> Vec<u8> {
        let mut data = Vec::with_capacity(samples.len() * 8);
        for s in samples {
            data.extend_from_slice(&(s.re as f32).to_le_bytes());
            data.extend_from_slice(&(s.im as f32).to_le_bytes());
        }
        data
    }

    fn serialize_i16(&self, samples: &[IQSample]) -> Vec<u8> {
        let mut data = Vec::with_capacity(samples.len() * 4);
        for s in samples {
            let i_raw = (s.re.clamp(-1.0, 1.0) * 32767.0) as i16;
            let q_raw = (s.im.clamp(-1.0, 1.0) * 32767.0) as i16;
            data.extend_from_slice(&i_raw.to_le_bytes());
            data.extend_from_slice(&q_raw.to_le_bytes());
        }
        data
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sample_format_parse() {
        assert_eq!(SampleFormat::from_str("f32"), Some(SampleFormat::Float32));
        assert_eq!(SampleFormat::from_str("float32"), Some(SampleFormat::Float32));
        assert_eq!(SampleFormat::from_str("i16"), Some(SampleFormat::Int16));
        assert_eq!(SampleFormat::from_str("int16"), Some(SampleFormat::Int16));
        assert_eq!(SampleFormat::from_str("invalid"), None);
    }

    #[test]
    fn test_sample_format_bytes() {
        assert_eq!(SampleFormat::Float32.bytes_per_sample(), 8);
        assert_eq!(SampleFormat::Int16.bytes_per_sample(), 4);
    }

    #[test]
    fn test_f32_parsing() {
        let receiver = BenchmarkReceiver {
            socket: UdpSocket::bind("0.0.0.0:0").unwrap(),
            format: SampleFormat::Float32,
            buffer: vec![],
            packets_received: 0,
            samples_received: 0,
            bytes_received: 0,
            errors: 0,
        };

        // Create test data: I=0.5, Q=-0.5
        let mut data = Vec::new();
        data.extend_from_slice(&0.5f32.to_le_bytes());
        data.extend_from_slice(&(-0.5f32).to_le_bytes());

        let samples = receiver.parse_f32(&data);
        assert_eq!(samples.len(), 1);
        assert!((samples[0].re - 0.5).abs() < 0.001);
        assert!((samples[0].im - (-0.5)).abs() < 0.001);
    }

    #[test]
    fn test_i16_parsing() {
        let receiver = BenchmarkReceiver {
            socket: UdpSocket::bind("0.0.0.0:0").unwrap(),
            format: SampleFormat::Int16,
            buffer: vec![],
            packets_received: 0,
            samples_received: 0,
            bytes_received: 0,
            errors: 0,
        };

        // Create test data: I=16384 (0.5), Q=-16384 (-0.5)
        let mut data = Vec::new();
        data.extend_from_slice(&16384i16.to_le_bytes());
        data.extend_from_slice(&(-16384i16).to_le_bytes());

        let samples = receiver.parse_i16(&data);
        assert_eq!(samples.len(), 1);
        assert!((samples[0].re - 0.5).abs() < 0.001);
        assert!((samples[0].im - (-0.5)).abs() < 0.001);
    }
}
