//! Packet Sink — Collect and count packets with statistics
//!
//! Terminal block that receives decoded packets and accumulates
//! statistics: total count, byte count, error rate, throughput,
//! and per-packet metadata. Useful for BER testing, protocol
//! validation, and performance measurement.
//! GNU Radio equivalent: packet sink / message debug.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::packet_sink::PacketSink;
//!
//! let mut sink = PacketSink::new();
//! sink.receive(b"Hello", true);
//! sink.receive(b"World", true);
//! sink.receive(b"Error", false);
//! assert_eq!(sink.total_packets(), 3);
//! assert_eq!(sink.good_packets(), 2);
//! assert_eq!(sink.error_packets(), 1);
//! ```

/// Statistics for a single received packet.
#[derive(Debug, Clone)]
pub struct PacketRecord {
    /// Packet sequence number (0-based).
    pub seq: u64,
    /// Packet data.
    pub data: Vec<u8>,
    /// Whether the packet passed CRC/validation.
    pub valid: bool,
    /// Optional RSSI or SNR.
    pub quality: Option<f64>,
}

/// Packet collector with statistics.
#[derive(Debug, Clone)]
pub struct PacketSink {
    /// All received packet records.
    records: Vec<PacketRecord>,
    /// Total packets received.
    total: u64,
    /// Packets that passed validation.
    good: u64,
    /// Packets that failed validation.
    bad: u64,
    /// Total bytes received (all packets).
    total_bytes: u64,
    /// Maximum packets to store (0 = unlimited).
    max_records: usize,
}

impl PacketSink {
    /// Create a new packet sink.
    pub fn new() -> Self {
        Self {
            records: Vec::new(),
            total: 0,
            good: 0,
            bad: 0,
            total_bytes: 0,
            max_records: 0,
        }
    }

    /// Create with a maximum record count (ring buffer behavior).
    pub fn with_max_records(max: usize) -> Self {
        Self {
            max_records: max,
            ..Self::new()
        }
    }

    /// Receive a packet.
    pub fn receive(&mut self, data: &[u8], valid: bool) {
        self.receive_with_quality(data, valid, None);
    }

    /// Receive a packet with quality metric.
    pub fn receive_with_quality(&mut self, data: &[u8], valid: bool, quality: Option<f64>) {
        let record = PacketRecord {
            seq: self.total,
            data: data.to_vec(),
            valid,
            quality,
        };

        self.total += 1;
        self.total_bytes += data.len() as u64;
        if valid {
            self.good += 1;
        } else {
            self.bad += 1;
        }

        if self.max_records > 0 && self.records.len() >= self.max_records {
            self.records.remove(0);
        }
        self.records.push(record);
    }

    /// Total packets received.
    pub fn total_packets(&self) -> u64 {
        self.total
    }

    /// Good (valid) packets.
    pub fn good_packets(&self) -> u64 {
        self.good
    }

    /// Error (invalid) packets.
    pub fn error_packets(&self) -> u64 {
        self.bad
    }

    /// Packet error rate (PER).
    pub fn packet_error_rate(&self) -> f64 {
        if self.total == 0 {
            return 0.0;
        }
        self.bad as f64 / self.total as f64
    }

    /// Packet success rate.
    pub fn packet_success_rate(&self) -> f64 {
        1.0 - self.packet_error_rate()
    }

    /// Total bytes received.
    pub fn total_bytes(&self) -> u64 {
        self.total_bytes
    }

    /// Average packet size.
    pub fn avg_packet_size(&self) -> f64 {
        if self.total == 0 {
            return 0.0;
        }
        self.total_bytes as f64 / self.total as f64
    }

    /// Average quality metric (e.g., SNR) of good packets.
    pub fn avg_quality(&self) -> Option<f64> {
        let quality_values: Vec<f64> = self.records
            .iter()
            .filter(|r| r.valid)
            .filter_map(|r| r.quality)
            .collect();
        if quality_values.is_empty() {
            None
        } else {
            Some(quality_values.iter().sum::<f64>() / quality_values.len() as f64)
        }
    }

    /// Get the last N received packets.
    pub fn last_n(&self, n: usize) -> &[PacketRecord] {
        let start = self.records.len().saturating_sub(n);
        &self.records[start..]
    }

    /// Get all stored records.
    pub fn records(&self) -> &[PacketRecord] {
        &self.records
    }

    /// Get the last received packet.
    pub fn last(&self) -> Option<&PacketRecord> {
        self.records.last()
    }

    /// Check if a specific byte pattern was received.
    pub fn contains_data(&self, data: &[u8]) -> bool {
        self.records.iter().any(|r| r.data == data)
    }

    /// Count packets matching a predicate.
    pub fn count_matching<F: Fn(&PacketRecord) -> bool>(&self, pred: F) -> usize {
        self.records.iter().filter(|r| pred(r)).count()
    }

    /// Clear all records and reset counters.
    pub fn clear(&mut self) {
        self.records.clear();
        self.total = 0;
        self.good = 0;
        self.bad = 0;
        self.total_bytes = 0;
    }
}

impl Default for PacketSink {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_receive() {
        let mut sink = PacketSink::new();
        sink.receive(b"test", true);
        assert_eq!(sink.total_packets(), 1);
        assert_eq!(sink.good_packets(), 1);
        assert_eq!(sink.error_packets(), 0);
    }

    #[test]
    fn test_error_packets() {
        let mut sink = PacketSink::new();
        sink.receive(b"good", true);
        sink.receive(b"bad", false);
        assert_eq!(sink.total_packets(), 2);
        assert_eq!(sink.good_packets(), 1);
        assert_eq!(sink.error_packets(), 1);
    }

    #[test]
    fn test_per() {
        let mut sink = PacketSink::new();
        for _ in 0..8 {
            sink.receive(b"ok", true);
        }
        for _ in 0..2 {
            sink.receive(b"err", false);
        }
        assert!((sink.packet_error_rate() - 0.2).abs() < 1e-10);
        assert!((sink.packet_success_rate() - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_byte_stats() {
        let mut sink = PacketSink::new();
        sink.receive(b"Hello", true);    // 5 bytes
        sink.receive(b"World!!", true);  // 7 bytes
        assert_eq!(sink.total_bytes(), 12);
        assert!((sink.avg_packet_size() - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_quality_metric() {
        let mut sink = PacketSink::new();
        sink.receive_with_quality(b"a", true, Some(10.0));
        sink.receive_with_quality(b"b", true, Some(20.0));
        sink.receive_with_quality(b"c", false, Some(5.0)); // Bad packet excluded
        let avg = sink.avg_quality().unwrap();
        assert!((avg - 15.0).abs() < 1e-10);
    }

    #[test]
    fn test_max_records() {
        let mut sink = PacketSink::with_max_records(3);
        for i in 0..5 {
            sink.receive(&[i as u8], true);
        }
        assert_eq!(sink.records().len(), 3);
        assert_eq!(sink.total_packets(), 5);
        assert_eq!(sink.records()[0].data, vec![2]); // Oldest kept
    }

    #[test]
    fn test_last_n() {
        let mut sink = PacketSink::new();
        for i in 0..10 {
            sink.receive(&[i as u8], true);
        }
        let last3 = sink.last_n(3);
        assert_eq!(last3.len(), 3);
        assert_eq!(last3[0].data, vec![7]);
    }

    #[test]
    fn test_last() {
        let mut sink = PacketSink::new();
        sink.receive(b"last", true);
        assert_eq!(sink.last().unwrap().data, b"last");
    }

    #[test]
    fn test_contains_data() {
        let mut sink = PacketSink::new();
        sink.receive(b"needle", true);
        sink.receive(b"haystack", true);
        assert!(sink.contains_data(b"needle"));
        assert!(!sink.contains_data(b"missing"));
    }

    #[test]
    fn test_count_matching() {
        let mut sink = PacketSink::new();
        sink.receive(b"short", true);
        sink.receive(b"medium msg", true);
        sink.receive(b"a very long message here", true);
        let long_count = sink.count_matching(|r| r.data.len() > 10);
        assert_eq!(long_count, 1);
    }

    #[test]
    fn test_clear() {
        let mut sink = PacketSink::new();
        sink.receive(b"test", true);
        sink.clear();
        assert_eq!(sink.total_packets(), 0);
        assert!(sink.records().is_empty());
    }

    #[test]
    fn test_empty() {
        let sink = PacketSink::new();
        assert_eq!(sink.total_packets(), 0);
        assert_eq!(sink.packet_error_rate(), 0.0);
        assert_eq!(sink.avg_packet_size(), 0.0);
        assert!(sink.avg_quality().is_none());
        assert!(sink.last().is_none());
    }

    #[test]
    fn test_default() {
        let sink = PacketSink::default();
        assert!(sink.records().is_empty());
    }

    #[test]
    fn test_sequence_numbers() {
        let mut sink = PacketSink::new();
        sink.receive(b"a", true);
        sink.receive(b"b", true);
        sink.receive(b"c", true);
        assert_eq!(sink.records()[0].seq, 0);
        assert_eq!(sink.records()[1].seq, 1);
        assert_eq!(sink.records()[2].seq, 2);
    }
}
