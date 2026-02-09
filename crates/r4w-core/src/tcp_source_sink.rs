//! TCP Source/Sink — Stream IQ samples over TCP
//!
//! Reliable, ordered IQ sample streaming for distributed SDR pipelines.
//! Supports client and server modes with configurable IQ formats.
//! GNU Radio equivalent: `tcp_source`, `tcp_sink`.
//!
//! ## Example
//!
//! ```rust,no_run
//! use r4w_core::tcp_source_sink::{TcpSink, TcpSource, TcpMode};
//!
//! // Server listens for connections
//! let mut sink = TcpSink::new("127.0.0.1:5000", TcpMode::Server);
//! sink.start().unwrap();
//!
//! // Client connects and reads
//! let mut source = TcpSource::new("127.0.0.1:5000", TcpMode::Client);
//! source.connect().unwrap();
//! ```

use num_complex::Complex64;
use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream, ToSocketAddrs};
use std::time::Duration;

/// TCP connection mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TcpMode {
    /// Act as TCP client (connect to server).
    Client,
    /// Act as TCP server (listen for connections).
    Server,
}

/// TCP IQ sample source (receiver).
#[derive(Debug)]
pub struct TcpSource {
    address: String,
    mode: TcpMode,
    stream: Option<TcpStream>,
    listener: Option<TcpListener>,
    buffer: Vec<u8>,
    buffer_size: usize,
    timeout: Option<Duration>,
}

impl TcpSource {
    /// Create a new TCP source.
    pub fn new(address: &str, mode: TcpMode) -> Self {
        Self {
            address: address.to_string(),
            mode,
            stream: None,
            listener: None,
            buffer: vec![0u8; 65536],
            buffer_size: 65536,
            timeout: Some(Duration::from_secs(5)),
        }
    }

    /// Set receive timeout.
    pub fn set_timeout(&mut self, timeout: Option<Duration>) {
        self.timeout = timeout;
    }

    /// Set buffer size.
    pub fn set_buffer_size(&mut self, size: usize) {
        self.buffer_size = size;
        self.buffer.resize(size, 0);
    }

    /// Connect or start listening.
    pub fn connect(&mut self) -> io::Result<()> {
        match self.mode {
            TcpMode::Client => {
                let addr = self.address.to_socket_addrs()?
                    .next()
                    .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "invalid address"))?;
                let stream = TcpStream::connect_timeout(&addr, Duration::from_secs(10))?;
                stream.set_read_timeout(self.timeout)?;
                stream.set_nodelay(true)?;
                self.stream = Some(stream);
            }
            TcpMode::Server => {
                let listener = TcpListener::bind(&self.address)?;
                listener.set_nonblocking(false)?;
                let (stream, _addr) = listener.accept()?;
                stream.set_read_timeout(self.timeout)?;
                stream.set_nodelay(true)?;
                self.stream = Some(stream);
                self.listener = Some(listener);
            }
        }
        Ok(())
    }

    /// Check if connected.
    pub fn is_connected(&self) -> bool {
        self.stream.is_some()
    }

    /// Receive complex f64 samples (16 bytes per sample: 8 re + 8 im).
    pub fn recv_cf64(&mut self, max_samples: usize) -> io::Result<Vec<Complex64>> {
        let stream = self.stream.as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "not connected"))?;

        let bytes_per_sample = 16;
        let max_bytes = max_samples * bytes_per_sample;
        let read_size = max_bytes.min(self.buffer.len());

        let n = stream.read(&mut self.buffer[..read_size])?;
        let num_samples = n / bytes_per_sample;
        let mut samples = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let offset = i * bytes_per_sample;
            let re = f64::from_le_bytes(self.buffer[offset..offset + 8].try_into().unwrap());
            let im = f64::from_le_bytes(self.buffer[offset + 8..offset + 16].try_into().unwrap());
            samples.push(Complex64::new(re, im));
        }
        Ok(samples)
    }

    /// Receive complex f32 samples (8 bytes per sample: 4 re + 4 im).
    pub fn recv_cf32(&mut self, max_samples: usize) -> io::Result<Vec<Complex64>> {
        let stream = self.stream.as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "not connected"))?;

        let bytes_per_sample = 8;
        let max_bytes = max_samples * bytes_per_sample;
        let read_size = max_bytes.min(self.buffer.len());

        let n = stream.read(&mut self.buffer[..read_size])?;
        let num_samples = n / bytes_per_sample;
        let mut samples = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let offset = i * bytes_per_sample;
            let re = f32::from_le_bytes(self.buffer[offset..offset + 4].try_into().unwrap()) as f64;
            let im = f32::from_le_bytes(self.buffer[offset + 4..offset + 8].try_into().unwrap()) as f64;
            samples.push(Complex64::new(re, im));
        }
        Ok(samples)
    }

    /// Receive raw bytes.
    pub fn recv_raw(&mut self, max_bytes: usize) -> io::Result<Vec<u8>> {
        let stream = self.stream.as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "not connected"))?;

        let read_size = max_bytes.min(self.buffer.len());
        let n = stream.read(&mut self.buffer[..read_size])?;
        Ok(self.buffer[..n].to_vec())
    }

    /// Close the connection.
    pub fn close(&mut self) {
        self.stream = None;
        self.listener = None;
    }
}

impl Drop for TcpSource {
    fn drop(&mut self) {
        self.close();
    }
}

/// TCP IQ sample sink (sender).
#[derive(Debug)]
pub struct TcpSink {
    address: String,
    mode: TcpMode,
    stream: Option<TcpStream>,
    listener: Option<TcpListener>,
    timeout: Option<Duration>,
}

impl TcpSink {
    /// Create a new TCP sink.
    pub fn new(address: &str, mode: TcpMode) -> Self {
        Self {
            address: address.to_string(),
            mode,
            stream: None,
            listener: None,
            timeout: Some(Duration::from_secs(5)),
        }
    }

    /// Set send timeout.
    pub fn set_timeout(&mut self, timeout: Option<Duration>) {
        self.timeout = timeout;
    }

    /// Connect or start listening.
    pub fn start(&mut self) -> io::Result<()> {
        match self.mode {
            TcpMode::Client => {
                let addr = self.address.to_socket_addrs()?
                    .next()
                    .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "invalid address"))?;
                let stream = TcpStream::connect_timeout(&addr, Duration::from_secs(10))?;
                stream.set_write_timeout(self.timeout)?;
                stream.set_nodelay(true)?;
                self.stream = Some(stream);
            }
            TcpMode::Server => {
                let listener = TcpListener::bind(&self.address)?;
                let (stream, _addr) = listener.accept()?;
                stream.set_write_timeout(self.timeout)?;
                stream.set_nodelay(true)?;
                self.stream = Some(stream);
                self.listener = Some(listener);
            }
        }
        Ok(())
    }

    /// Check if connected.
    pub fn is_connected(&self) -> bool {
        self.stream.is_some()
    }

    /// Send complex f64 samples (16 bytes per sample).
    pub fn send_cf64(&mut self, samples: &[Complex64]) -> io::Result<()> {
        let stream = self.stream.as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "not connected"))?;

        let mut buf = Vec::with_capacity(samples.len() * 16);
        for s in samples {
            buf.extend_from_slice(&s.re.to_le_bytes());
            buf.extend_from_slice(&s.im.to_le_bytes());
        }
        stream.write_all(&buf)
    }

    /// Send complex f32 samples (8 bytes per sample).
    pub fn send_cf32(&mut self, samples: &[Complex64]) -> io::Result<()> {
        let stream = self.stream.as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "not connected"))?;

        let mut buf = Vec::with_capacity(samples.len() * 8);
        for s in samples {
            buf.extend_from_slice(&(s.re as f32).to_le_bytes());
            buf.extend_from_slice(&(s.im as f32).to_le_bytes());
        }
        stream.write_all(&buf)
    }

    /// Send raw bytes.
    pub fn send_raw(&mut self, data: &[u8]) -> io::Result<()> {
        let stream = self.stream.as_mut()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "not connected"))?;
        stream.write_all(data)
    }

    /// Flush the stream.
    pub fn flush(&mut self) -> io::Result<()> {
        if let Some(stream) = self.stream.as_mut() {
            stream.flush()
        } else {
            Ok(())
        }
    }

    /// Close the connection.
    pub fn close(&mut self) {
        self.stream = None;
        self.listener = None;
    }
}

impl Drop for TcpSink {
    fn drop(&mut self) {
        self.close();
    }
}

/// Encode Complex64 samples to interleaved f64 LE bytes.
pub fn samples_to_cf64_bytes(samples: &[Complex64]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(samples.len() * 16);
    for s in samples {
        buf.extend_from_slice(&s.re.to_le_bytes());
        buf.extend_from_slice(&s.im.to_le_bytes());
    }
    buf
}

/// Decode interleaved f64 LE bytes to Complex64 samples.
pub fn cf64_bytes_to_samples(data: &[u8]) -> Vec<Complex64> {
    let num = data.len() / 16;
    let mut samples = Vec::with_capacity(num);
    for i in 0..num {
        let off = i * 16;
        let re = f64::from_le_bytes(data[off..off + 8].try_into().unwrap());
        let im = f64::from_le_bytes(data[off + 8..off + 16].try_into().unwrap());
        samples.push(Complex64::new(re, im));
    }
    samples
}

/// Encode Complex64 samples to interleaved f32 LE bytes.
pub fn samples_to_cf32_bytes(samples: &[Complex64]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(samples.len() * 8);
    for s in samples {
        buf.extend_from_slice(&(s.re as f32).to_le_bytes());
        buf.extend_from_slice(&(s.im as f32).to_le_bytes());
    }
    buf
}

/// Decode interleaved f32 LE bytes to Complex64 samples.
pub fn cf32_bytes_to_samples(data: &[u8]) -> Vec<Complex64> {
    let num = data.len() / 8;
    let mut samples = Vec::with_capacity(num);
    for i in 0..num {
        let off = i * 8;
        let re = f32::from_le_bytes(data[off..off + 4].try_into().unwrap()) as f64;
        let im = f32::from_le_bytes(data[off + 4..off + 8].try_into().unwrap()) as f64;
        samples.push(Complex64::new(re, im));
    }
    samples
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cf64_roundtrip() {
        let samples = vec![
            Complex64::new(1.0, -2.0),
            Complex64::new(3.5, 4.5),
            Complex64::new(-0.1, 0.2),
        ];
        let bytes = samples_to_cf64_bytes(&samples);
        assert_eq!(bytes.len(), 48);
        let recovered = cf64_bytes_to_samples(&bytes);
        assert_eq!(samples, recovered);
    }

    #[test]
    fn test_cf32_roundtrip() {
        let samples = vec![
            Complex64::new(1.0, -2.0),
            Complex64::new(3.5, 4.5),
        ];
        let bytes = samples_to_cf32_bytes(&samples);
        assert_eq!(bytes.len(), 16);
        let recovered = cf32_bytes_to_samples(&bytes);
        for (a, b) in samples.iter().zip(recovered.iter()) {
            assert!((a.re - b.re).abs() < 1e-6);
            assert!((a.im - b.im).abs() < 1e-6);
        }
    }

    #[test]
    fn test_cf64_empty() {
        let bytes = samples_to_cf64_bytes(&[]);
        assert_eq!(bytes.len(), 0);
        let recovered = cf64_bytes_to_samples(&bytes);
        assert!(recovered.is_empty());
    }

    #[test]
    fn test_cf32_empty() {
        let bytes = samples_to_cf32_bytes(&[]);
        assert_eq!(bytes.len(), 0);
        let recovered = cf32_bytes_to_samples(&bytes);
        assert!(recovered.is_empty());
    }

    #[test]
    fn test_tcp_source_creation() {
        let source = TcpSource::new("127.0.0.1:5000", TcpMode::Client);
        assert!(!source.is_connected());
        assert_eq!(source.address, "127.0.0.1:5000");
    }

    #[test]
    fn test_tcp_sink_creation() {
        let sink = TcpSink::new("127.0.0.1:5001", TcpMode::Server);
        assert!(!sink.is_connected());
    }

    #[test]
    fn test_tcp_loopback() {
        // Server sink listens, client source connects
        let addr = "127.0.0.1:0"; // OS-assigned port

        // Bind to get the actual port
        let listener = TcpListener::bind(addr).unwrap();
        let actual_addr = listener.local_addr().unwrap();
        let port = actual_addr.port();

        // Spawn a thread to accept and send data
        let handle = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let samples = vec![
                Complex64::new(1.0, 2.0),
                Complex64::new(3.0, 4.0),
                Complex64::new(5.0, 6.0),
            ];
            let bytes = samples_to_cf64_bytes(&samples);
            stream.write_all(&bytes).unwrap();
            stream.flush().unwrap();
        });

        // Give server time to listen
        std::thread::sleep(Duration::from_millis(50));

        let mut source = TcpSource::new(&format!("127.0.0.1:{}", port), TcpMode::Client);
        source.connect().unwrap();
        assert!(source.is_connected());

        let received = source.recv_cf64(10).unwrap();
        assert_eq!(received.len(), 3);
        assert_eq!(received[0], Complex64::new(1.0, 2.0));
        assert_eq!(received[1], Complex64::new(3.0, 4.0));
        assert_eq!(received[2], Complex64::new(5.0, 6.0));

        handle.join().unwrap();
    }

    #[test]
    fn test_tcp_cf32_loopback() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();

        let handle = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let samples = vec![Complex64::new(1.5, -2.5), Complex64::new(0.0, 1.0)];
            let bytes = samples_to_cf32_bytes(&samples);
            stream.write_all(&bytes).unwrap();
            stream.flush().unwrap();
        });

        std::thread::sleep(Duration::from_millis(50));

        let mut source = TcpSource::new(&format!("127.0.0.1:{}", port), TcpMode::Client);
        source.connect().unwrap();

        let received = source.recv_cf32(10).unwrap();
        assert_eq!(received.len(), 2);
        assert!((received[0].re - 1.5).abs() < 1e-6);
        assert!((received[0].im - (-2.5)).abs() < 1e-6);

        handle.join().unwrap();
    }

    #[test]
    fn test_tcp_sink_send() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();

        let handle = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut buf = vec![0u8; 48];
            stream.read_exact(&mut buf).unwrap();
            let samples = cf64_bytes_to_samples(&buf);
            assert_eq!(samples.len(), 3);
            assert_eq!(samples[0], Complex64::new(10.0, 20.0));
            samples
        });

        std::thread::sleep(Duration::from_millis(50));

        let mut sink = TcpSink::new(&format!("127.0.0.1:{}", port), TcpMode::Client);
        sink.start().unwrap();
        assert!(sink.is_connected());

        let samples = vec![
            Complex64::new(10.0, 20.0),
            Complex64::new(30.0, 40.0),
            Complex64::new(50.0, 60.0),
        ];
        sink.send_cf64(&samples).unwrap();
        sink.flush().unwrap();

        let result = handle.join().unwrap();
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn test_partial_read() {
        // When bytes don't align to sample boundary, partial data is truncated
        let data = vec![0u8; 20]; // 20 bytes = 1 full cf64 sample + 4 leftover
        let samples = cf64_bytes_to_samples(&data);
        assert_eq!(samples.len(), 1); // Only 1 complete sample
    }

    #[test]
    fn test_mode_enum() {
        assert_eq!(TcpMode::Client, TcpMode::Client);
        assert_ne!(TcpMode::Client, TcpMode::Server);
    }

    #[test]
    fn test_source_close() {
        let mut source = TcpSource::new("127.0.0.1:5555", TcpMode::Client);
        source.close();
        assert!(!source.is_connected());
    }

    #[test]
    fn test_sink_close() {
        let mut sink = TcpSink::new("127.0.0.1:5556", TcpMode::Client);
        sink.close();
        assert!(!sink.is_connected());
    }

    #[test]
    fn test_set_buffer_size() {
        let mut source = TcpSource::new("127.0.0.1:5557", TcpMode::Client);
        source.set_buffer_size(1024);
        assert_eq!(source.buffer.len(), 1024);
    }
}
