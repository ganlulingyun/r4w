//! # MIMO Detection — Sphere Decoder and K-Best
//!
//! Near-optimal MIMO spatial multiplexing detection algorithms including
//! sphere decoder (Schnorr-Euchner), K-best tree search, exhaustive ML,
//! and MMSE-SIC. Essential for MIMO-OFDM (LTE, 5G NR, Wi-Fi 802.11n/ac/ax).
//!
//! GNU Radio equivalent: `gr-mimo` ML/sphere-decoder, `gr-lte` MIMO detection.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::mimo_detector::*;
//! use num_complex::Complex64;
//!
//! let config = MimoDetectorConfig {
//!     algorithm: DetectionAlgorithm::SphereDecoder { initial_radius: 10.0 },
//!     num_tx: 2,
//!     num_rx: 2,
//!     constellation: ConstellationSet::qpsk(),
//!     noise_variance: 0.1,
//! };
//! let detector = MimoDetector::new(config);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// MIMO detection algorithm selection.
#[derive(Debug, Clone)]
pub enum DetectionAlgorithm {
    /// Schnorr-Euchner sphere decoder with initial radius.
    SphereDecoder { initial_radius: f64 },
    /// K-best tree search keeping top-K candidates per level.
    KBest { k: usize },
    /// Maximum likelihood (exhaustive search).
    MaximumLikelihood,
    /// MMSE-SIC (ordered successive interference cancellation).
    MmseSic,
}

/// Constellation definition for detection.
#[derive(Debug, Clone)]
pub struct ConstellationSet {
    /// Constellation points.
    pub points: Vec<Complex64>,
    /// Bits per symbol.
    pub bits_per_symbol: usize,
    /// Gray-coded bit labels.
    pub label_map: Vec<u64>,
}

impl ConstellationSet {
    /// QPSK constellation.
    pub fn qpsk() -> Self {
        let norm = 1.0 / 2.0_f64.sqrt();
        let points = vec![
            Complex64::new(norm, norm),
            Complex64::new(-norm, norm),
            Complex64::new(-norm, -norm),
            Complex64::new(norm, -norm),
        ];
        Self {
            points,
            bits_per_symbol: 2,
            label_map: vec![0, 1, 3, 2], // Gray code
        }
    }

    /// 16-QAM constellation.
    pub fn qam16() -> Self {
        let norm = 1.0 / 10.0_f64.sqrt();
        let mut points = Vec::with_capacity(16);
        let mut labels = Vec::with_capacity(16);
        let gray4 = [0u64, 1, 3, 2];
        for (i, &gi) in gray4.iter().enumerate() {
            for (j, &gj) in gray4.iter().enumerate() {
                let re = (2.0 * i as f64 - 3.0) * norm;
                let im = (2.0 * j as f64 - 3.0) * norm;
                points.push(Complex64::new(re, im));
                labels.push((gi << 2) | gj);
            }
        }
        Self {
            points,
            bits_per_symbol: 4,
            label_map: labels,
        }
    }

    /// 64-QAM constellation.
    pub fn qam64() -> Self {
        let norm = 1.0 / 42.0_f64.sqrt();
        let mut points = Vec::with_capacity(64);
        let mut labels = Vec::with_capacity(64);
        let gray8 = [0u64, 1, 3, 2, 6, 7, 5, 4];
        for (i, &gi) in gray8.iter().enumerate() {
            for (j, &gj) in gray8.iter().enumerate() {
                let re = (2.0 * i as f64 - 7.0) * norm;
                let im = (2.0 * j as f64 - 7.0) * norm;
                points.push(Complex64::new(re, im));
                labels.push((gi << 3) | gj);
            }
        }
        Self {
            points,
            bits_per_symbol: 6,
            label_map: labels,
        }
    }

    /// 256-QAM constellation.
    pub fn qam256() -> Self {
        let norm = 1.0 / 170.0_f64.sqrt();
        let mut points = Vec::with_capacity(256);
        let mut labels = Vec::with_capacity(256);
        let gray16 = [0u64, 1, 3, 2, 6, 7, 5, 4, 12, 13, 15, 14, 10, 11, 9, 8];
        for (i, &gi) in gray16.iter().enumerate() {
            for (j, &gj) in gray16.iter().enumerate() {
                let re = (2.0 * i as f64 - 15.0) * norm;
                let im = (2.0 * j as f64 - 15.0) * norm;
                points.push(Complex64::new(re, im));
                labels.push((gi << 4) | gj);
            }
        }
        Self {
            points,
            bits_per_symbol: 8,
            label_map: labels,
        }
    }
}

/// MIMO detector configuration.
#[derive(Debug, Clone)]
pub struct MimoDetectorConfig {
    /// Detection algorithm.
    pub algorithm: DetectionAlgorithm,
    /// Number of transmit antennas.
    pub num_tx: usize,
    /// Number of receive antennas.
    pub num_rx: usize,
    /// Constellation set.
    pub constellation: ConstellationSet,
    /// Noise variance.
    pub noise_variance: f64,
}

/// Complex matrix stored row-major.
#[derive(Debug, Clone)]
pub struct ChannelMatrix {
    pub data: Vec<Complex64>,
    pub rows: usize,
    pub cols: usize,
}

impl ChannelMatrix {
    pub fn new(rows: usize, cols: usize, data: Vec<Complex64>) -> Self {
        assert_eq!(data.len(), rows * cols);
        Self { data, rows, cols }
    }

    pub fn identity(n: usize) -> Self {
        let mut data = vec![Complex64::new(0.0, 0.0); n * n];
        for i in 0..n {
            data[i * n + i] = Complex64::new(1.0, 0.0);
        }
        Self { data, rows: n, cols: n }
    }

    fn get(&self, r: usize, c: usize) -> Complex64 {
        self.data[r * self.cols + c]
    }

    fn set(&mut self, r: usize, c: usize, val: Complex64) {
        self.data[r * self.cols + c] = val;
    }

    fn hermitian(&self) -> Self {
        let mut out = vec![Complex64::new(0.0, 0.0); self.rows * self.cols];
        for r in 0..self.rows {
            for c in 0..self.cols {
                out[c * self.rows + r] = self.get(r, c).conj();
            }
        }
        Self { data: out, rows: self.cols, cols: self.rows }
    }

    fn mat_mul(&self, other: &Self) -> Self {
        assert_eq!(self.cols, other.rows);
        let mut out = vec![Complex64::new(0.0, 0.0); self.rows * other.cols];
        for r in 0..self.rows {
            for c in 0..other.cols {
                let mut sum = Complex64::new(0.0, 0.0);
                for k in 0..self.cols {
                    sum += self.get(r, k) * other.get(k, c);
                }
                out[r * other.cols + c] = sum;
            }
        }
        Self { data: out, rows: self.rows, cols: other.cols }
    }

    fn mat_vec_mul(&self, v: &[Complex64]) -> Vec<Complex64> {
        assert_eq!(self.cols, v.len());
        let mut out = vec![Complex64::new(0.0, 0.0); self.rows];
        for r in 0..self.rows {
            for c in 0..self.cols {
                out[r] += self.get(r, c) * v[c];
            }
        }
        out
    }
}

/// Detection result for one symbol vector.
#[derive(Debug, Clone)]
pub struct DetectionResult {
    /// Detected constellation symbols.
    pub detected_symbols: Vec<Complex64>,
    /// Hard-decided bits.
    pub hard_bits: Vec<bool>,
    /// Soft LLR values for each bit.
    pub soft_bits: Vec<f64>,
    /// Euclidean distance metric.
    pub metric: f64,
    /// Number of nodes visited (complexity).
    pub nodes_visited: usize,
}

/// The MIMO detector.
pub struct MimoDetector {
    config: MimoDetectorConfig,
    total_nodes: u64,
    total_detections: u64,
}

impl MimoDetector {
    /// Create a new MIMO detector.
    pub fn new(config: MimoDetectorConfig) -> Self {
        Self {
            config,
            total_nodes: 0,
            total_detections: 0,
        }
    }

    /// Detect transmitted symbols from received vector.
    pub fn detect(
        &mut self,
        received: &[Complex64],
        channel: &ChannelMatrix,
    ) -> DetectionResult {
        let result = match &self.config.algorithm {
            DetectionAlgorithm::SphereDecoder { initial_radius } => {
                self.sphere_decode(received, channel, *initial_radius)
            }
            DetectionAlgorithm::KBest { k } => {
                self.k_best_detect(received, channel, *k)
            }
            DetectionAlgorithm::MaximumLikelihood => {
                self.ml_detect(received, channel)
            }
            DetectionAlgorithm::MmseSic => {
                self.mmse_sic_detect(received, channel)
            }
        };
        self.total_nodes += result.nodes_visited as u64;
        self.total_detections += 1;
        result
    }

    /// Detect a batch of received vectors.
    pub fn detect_batch(
        &mut self,
        received: &[Vec<Complex64>],
        channel: &ChannelMatrix,
    ) -> Vec<DetectionResult> {
        received.iter().map(|r| self.detect(r, channel)).collect()
    }

    /// Compute soft LLR values.
    pub fn detect_soft(
        &mut self,
        received: &[Complex64],
        channel: &ChannelMatrix,
    ) -> Vec<f64> {
        let result = self.detect(received, channel);
        result.soft_bits
    }

    /// Average number of nodes visited per detection.
    pub fn average_complexity(&self) -> f64 {
        if self.total_detections == 0 { 0.0 }
        else { self.total_nodes as f64 / self.total_detections as f64 }
    }

    fn ml_detect(&self, received: &[Complex64], channel: &ChannelMatrix) -> DetectionResult {
        let nt = self.config.num_tx;
        let constellation = &self.config.constellation.points;
        let m = constellation.len();
        let total = m.pow(nt as u32);

        let mut best_metric = f64::MAX;
        let mut best_symbols = vec![Complex64::new(0.0, 0.0); nt];
        let mut nodes = 0usize;

        for idx in 0..total {
            let mut candidate = vec![Complex64::new(0.0, 0.0); nt];
            let mut tmp = idx;
            for t in 0..nt {
                candidate[t] = constellation[tmp % m];
                tmp /= m;
            }

            let hx = channel.mat_vec_mul(&candidate);
            let mut metric = 0.0;
            for (r, h) in received.iter().zip(hx.iter()) {
                metric += (r - h).norm_sqr();
            }
            nodes += 1;

            if metric < best_metric {
                best_metric = metric;
                best_symbols = candidate;
            }
        }

        // Compute soft LLR using max-log approximation
        let soft_bits = self.compute_soft_llr(received, channel, &best_symbols);
        let hard_bits = self.symbols_to_bits(&best_symbols);

        DetectionResult {
            detected_symbols: best_symbols,
            hard_bits,
            soft_bits,
            metric: best_metric,
            nodes_visited: nodes,
        }
    }

    fn sphere_decode(&self, received: &[Complex64], channel: &ChannelMatrix, initial_radius: f64) -> DetectionResult {
        let nt = self.config.num_tx;
        let (q, r) = qr_decompose(channel);

        // z = Q^H * y
        let qh = q.hermitian();
        let z = qh.mat_vec_mul(received);

        let constellation = &self.config.constellation.points;
        let mut best_metric = initial_radius * initial_radius;
        let mut best_symbols = vec![Complex64::new(0.0, 0.0); nt];
        let mut nodes = 0usize;

        // Stack-based DFS sphere decoder
        let mut stack: Vec<(usize, Vec<Complex64>, f64)> = Vec::new();

        // Start from the top layer (nt-1)
        for s in constellation.iter() {
            let partial = r.get(nt - 1, nt - 1) * s;
            let diff = z[nt - 1] - partial;
            let pd = diff.norm_sqr();
            if pd <= best_metric {
                let mut syms = vec![Complex64::new(0.0, 0.0); nt];
                syms[nt - 1] = *s;
                stack.push((nt - 1, syms, pd));
            }
            nodes += 1;
        }

        while let Some((layer, syms, partial_dist)) = stack.pop() {
            if layer == 0 {
                if partial_dist < best_metric {
                    best_metric = partial_dist;
                    best_symbols = syms;
                }
                continue;
            }

            let next = layer - 1;
            for s in constellation.iter() {
                let mut sum = r.get(next, next) * s;
                for k in (next + 1)..nt {
                    sum += r.get(next, k) * syms[k];
                }
                let diff = z[next] - sum;
                let new_dist = partial_dist + diff.norm_sqr();
                nodes += 1;

                if new_dist <= best_metric {
                    let mut new_syms = syms.clone();
                    new_syms[next] = *s;
                    if next == 0 {
                        if new_dist < best_metric {
                            best_metric = new_dist;
                            best_symbols = new_syms;
                        }
                    } else {
                        stack.push((next, new_syms, new_dist));
                    }
                }
            }
        }

        let soft_bits = self.compute_soft_llr(received, channel, &best_symbols);
        let hard_bits = self.symbols_to_bits(&best_symbols);

        DetectionResult {
            detected_symbols: best_symbols,
            hard_bits,
            soft_bits,
            metric: best_metric,
            nodes_visited: nodes,
        }
    }

    fn k_best_detect(&self, received: &[Complex64], channel: &ChannelMatrix, k: usize) -> DetectionResult {
        let nt = self.config.num_tx;
        let (q, r) = qr_decompose(channel);
        let qh = q.hermitian();
        let z = qh.mat_vec_mul(received);
        let constellation = &self.config.constellation.points;
        let mut nodes = 0usize;

        // paths: (symbols, partial_metric)
        let mut paths: Vec<(Vec<Complex64>, f64)> = Vec::new();

        // Start from top layer
        for s in constellation.iter() {
            let partial = r.get(nt - 1, nt - 1) * s;
            let diff = z[nt - 1] - partial;
            let pd = diff.norm_sqr();
            let mut syms = vec![Complex64::new(0.0, 0.0); nt];
            syms[nt - 1] = *s;
            paths.push((syms, pd));
            nodes += 1;
        }

        // Keep only K best
        paths.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        paths.truncate(k);

        // Expand through remaining layers
        for layer in (0..nt - 1).rev() {
            let mut new_paths = Vec::new();
            for (syms, pd) in &paths {
                for s in constellation.iter() {
                    let mut sum = r.get(layer, layer) * s;
                    for m in (layer + 1)..nt {
                        sum += r.get(layer, m) * syms[m];
                    }
                    let diff = z[layer] - sum;
                    let new_dist = pd + diff.norm_sqr();
                    let mut new_syms = syms.clone();
                    new_syms[layer] = *s;
                    new_paths.push((new_syms, new_dist));
                    nodes += 1;
                }
            }
            new_paths.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
            new_paths.truncate(k);
            paths = new_paths;
        }

        let (best_symbols, best_metric) = paths.into_iter().next().unwrap_or_else(|| {
            (vec![Complex64::new(0.0, 0.0); nt], f64::MAX)
        });

        let soft_bits = self.compute_soft_llr(received, channel, &best_symbols);
        let hard_bits = self.symbols_to_bits(&best_symbols);

        DetectionResult {
            detected_symbols: best_symbols,
            hard_bits,
            soft_bits,
            metric: best_metric,
            nodes_visited: nodes,
        }
    }

    fn mmse_sic_detect(&self, received: &[Complex64], channel: &ChannelMatrix) -> DetectionResult {
        let nt = self.config.num_tx;
        let nr = self.config.num_rx;
        let constellation = &self.config.constellation.points;
        let sigma2 = self.config.noise_variance;
        let mut nodes = 0usize;

        let mut detected = vec![Complex64::new(0.0, 0.0); nt];
        let mut y = received.to_vec();
        let mut h = channel.clone();
        let mut detected_order = vec![false; nt];

        for _iter in 0..nt {
            // Compute MMSE filter for each undetected stream
            let hh = h.hermitian();
            let hhh = hh.mat_mul(&h);

            // Find stream with highest post-MMSE SINR
            let mut best_stream = 0;
            let mut best_sinr = -f64::MAX;

            for t in 0..nt {
                if detected_order[t] { continue; }
                // Approximate: use diagonal of (H^H H + sigma2 I)^{-1}
                let diag = hhh.get(t, t).re + sigma2;
                let sinr = hhh.get(t, t).re / (diag - hhh.get(t, t).re + 1e-30);
                if sinr > best_sinr {
                    best_sinr = sinr;
                    best_stream = t;
                }
                nodes += 1;
            }

            // MMSE weight for the chosen stream
            let mut w = vec![Complex64::new(0.0, 0.0); nr];
            let col: Vec<Complex64> = (0..nr).map(|r| h.get(r, best_stream)).collect();
            let inv_diag = 1.0 / (hhh.get(best_stream, best_stream).re + sigma2);
            for r in 0..nr {
                w[r] = col[r].conj() * inv_diag;
            }

            // Filter
            let mut z = Complex64::new(0.0, 0.0);
            for r in 0..nr {
                z += w[r] * y[r];
            }

            // Slice to nearest constellation point
            let mut min_dist = f64::MAX;
            let mut best_sym = constellation[0];
            for &s in constellation.iter() {
                let d = (z - s).norm_sqr();
                if d < min_dist {
                    min_dist = d;
                    best_sym = s;
                }
                nodes += 1;
            }

            detected[best_stream] = best_sym;
            detected_order[best_stream] = true;

            // Cancel contribution
            for r in 0..nr {
                y[r] -= h.get(r, best_stream) * best_sym;
            }
        }

        // Compute total metric
        let hx = channel.mat_vec_mul(&detected);
        let mut metric = 0.0;
        for (r, h) in received.iter().zip(hx.iter()) {
            metric += (r - h).norm_sqr();
        }

        let soft_bits = self.compute_soft_llr(received, channel, &detected);
        let hard_bits = self.symbols_to_bits(&detected);

        DetectionResult {
            detected_symbols: detected,
            hard_bits,
            soft_bits,
            metric,
            nodes_visited: nodes,
        }
    }

    fn symbols_to_bits(&self, symbols: &[Complex64]) -> Vec<bool> {
        let constellation = &self.config.constellation;
        let mut bits = Vec::new();
        for s in symbols {
            // Find closest constellation point
            let mut min_dist = f64::MAX;
            let mut min_idx = 0;
            for (i, &p) in constellation.points.iter().enumerate() {
                let d = (s - p).norm_sqr();
                if d < min_dist {
                    min_dist = d;
                    min_idx = i;
                }
            }
            let label = constellation.label_map[min_idx];
            for b in (0..constellation.bits_per_symbol).rev() {
                bits.push(((label >> b) & 1) != 0);
            }
        }
        bits
    }

    fn compute_soft_llr(&self, received: &[Complex64], channel: &ChannelMatrix, detected: &[Complex64]) -> Vec<f64> {
        let nt = self.config.num_tx;
        let total_bits = nt * self.config.constellation.bits_per_symbol;
        let sigma2 = if self.config.noise_variance > 0.0 { self.config.noise_variance } else { 1.0 };
        let constellation = &self.config.constellation;

        // Compute ML metric for detected symbols
        let hx_best = channel.mat_vec_mul(detected);
        let metric_best: f64 = received.iter().zip(hx_best.iter())
            .map(|(r, h)| (r - h).norm_sqr()).sum();

        let hard_bits = self.symbols_to_bits(detected);
        let mut llrs = vec![0.0; total_bits];

        // For each bit position, compute LLR by flipping the corresponding
        // symbol and finding the nearest alternative constellation point
        for (b, llr) in llrs.iter_mut().enumerate() {
            let stream_idx = b / constellation.bits_per_symbol;
            let bit_in_sym = b % constellation.bits_per_symbol;

            // Find metric for best hypothesis with bit flipped
            let current_label = constellation.label_map.iter()
                .enumerate()
                .min_by(|(_, _), (_, _)| {
                    // Find the label matching detected[stream_idx]
                    std::cmp::Ordering::Equal
                })
                .map(|(i, _)| i)
                .unwrap_or(0);

            // Find the closest constellation point to detected[stream_idx]
            let mut min_dist = f64::MAX;
            let mut det_label_idx = 0;
            for (i, &p) in constellation.points.iter().enumerate() {
                let d = (detected[stream_idx] - p).norm_sqr();
                if d < min_dist {
                    min_dist = d;
                    det_label_idx = i;
                }
            }
            let det_label = constellation.label_map[det_label_idx];
            let det_bit = ((det_label >> (constellation.bits_per_symbol - 1 - bit_in_sym)) & 1) != 0;

            // Find the best alternative symbol with the opposite bit value
            let mut best_alt_metric = f64::MAX;
            for (i, &p) in constellation.points.iter().enumerate() {
                let label = constellation.label_map[i];
                let this_bit = ((label >> (constellation.bits_per_symbol - 1 - bit_in_sym)) & 1) != 0;
                if this_bit != det_bit {
                    // Substitute this symbol and compute metric
                    let mut alt_syms = detected.to_vec();
                    alt_syms[stream_idx] = p;
                    let hx_alt = channel.mat_vec_mul(&alt_syms);
                    let metric_alt: f64 = received.iter().zip(hx_alt.iter())
                        .map(|(r, h)| (r - h).norm_sqr()).sum();
                    if metric_alt < best_alt_metric {
                        best_alt_metric = metric_alt;
                    }
                }
            }

            // LLR = (1/sigma2) * (metric_for_bit_1 - metric_for_bit_0)
            let (metric_0, metric_1) = if det_bit {
                (best_alt_metric, metric_best)
            } else {
                (metric_best, best_alt_metric)
            };
            *llr = (metric_1 - metric_0) / sigma2;
        }
        llrs
    }
}

/// QR decomposition using modified Gram-Schmidt.
pub fn qr_decompose(h: &ChannelMatrix) -> (ChannelMatrix, ChannelMatrix) {
    let m = h.rows;
    let n = h.cols;
    let mut q_data = vec![Complex64::new(0.0, 0.0); m * n];
    let mut r_data = vec![Complex64::new(0.0, 0.0); n * n];

    // Copy columns
    let mut cols: Vec<Vec<Complex64>> = Vec::new();
    for j in 0..n {
        let col: Vec<Complex64> = (0..m).map(|i| h.get(i, j)).collect();
        cols.push(col);
    }

    let mut q_cols: Vec<Vec<Complex64>> = vec![vec![Complex64::new(0.0, 0.0); m]; n];

    for j in 0..n {
        let mut v = cols[j].clone();

        // Orthogonalize against previous columns
        for i in 0..j {
            let mut dot = Complex64::new(0.0, 0.0);
            for k in 0..m {
                dot += q_cols[i][k].conj() * v[k];
            }
            r_data[i * n + j] = dot;
            for k in 0..m {
                v[k] -= dot * q_cols[i][k];
            }
        }

        let norm: f64 = v.iter().map(|x| x.norm_sqr()).sum::<f64>().sqrt();
        r_data[j * n + j] = Complex64::new(norm, 0.0);

        if norm > 1e-15 {
            for k in 0..m {
                q_cols[j][k] = v[k] / norm;
            }
        }
    }

    // Pack Q
    for j in 0..n {
        for i in 0..m {
            q_data[i * n + j] = q_cols[j][i];
        }
    }

    (
        ChannelMatrix::new(m, n, q_data),
        ChannelMatrix::new(n, n, r_data),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn random_complex(seed: &mut u64) -> Complex64 {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        let re = (*seed as f64 / u64::MAX as f64) * 2.0 - 1.0;
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        let im = (*seed as f64 / u64::MAX as f64) * 2.0 - 1.0;
        Complex64::new(re, im)
    }

    #[test]
    fn test_2x2_qpsk_identity_channel() {
        let config = MimoDetectorConfig {
            algorithm: DetectionAlgorithm::SphereDecoder { initial_radius: 100.0 },
            num_tx: 2, num_rx: 2,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.001,
        };
        let mut detector = MimoDetector::new(config);

        let c = ConstellationSet::qpsk();
        let tx = vec![c.points[0], c.points[2]]; // two QPSK symbols
        let h = ChannelMatrix::identity(2);
        let received = h.mat_vec_mul(&tx);

        let result = detector.detect(&received, &h);
        for (det, orig) in result.detected_symbols.iter().zip(tx.iter()) {
            assert!((det - orig).norm() < 0.1, "Mismatch: {:?} vs {:?}", det, orig);
        }
        assert!(result.metric < 0.01);
    }

    #[test]
    fn test_ml_exhaustive_2x2_qpsk() {
        let config = MimoDetectorConfig {
            algorithm: DetectionAlgorithm::MaximumLikelihood,
            num_tx: 2, num_rx: 2,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.1,
        };
        let mut detector = MimoDetector::new(config);

        let c = ConstellationSet::qpsk();
        let h = ChannelMatrix::new(2, 2, vec![
            Complex64::new(0.8, 0.2), Complex64::new(-0.3, 0.5),
            Complex64::new(0.1, -0.4), Complex64::new(0.9, 0.1),
        ]);

        let tx = vec![c.points[1], c.points[3]];
        let received = h.mat_vec_mul(&tx);

        let result = detector.detect(&received, &h);
        // With no noise, ML should find exact solution
        for (det, orig) in result.detected_symbols.iter().zip(tx.iter()) {
            assert!((det - orig).norm() < 0.1);
        }
        assert_eq!(result.nodes_visited, 16); // 4^2
    }

    #[test]
    fn test_sphere_decoder_matches_ml() {
        let c = ConstellationSet::qpsk();
        let h = ChannelMatrix::new(2, 2, vec![
            Complex64::new(0.7, -0.3), Complex64::new(0.4, 0.6),
            Complex64::new(-0.2, 0.8), Complex64::new(0.5, -0.1),
        ]);

        let tx = vec![c.points[0], c.points[2]];
        let received = h.mat_vec_mul(&tx);

        let mut ml_det = MimoDetector::new(MimoDetectorConfig {
            algorithm: DetectionAlgorithm::MaximumLikelihood,
            num_tx: 2, num_rx: 2,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.01,
        });

        let mut sd_det = MimoDetector::new(MimoDetectorConfig {
            algorithm: DetectionAlgorithm::SphereDecoder { initial_radius: 100.0 },
            num_tx: 2, num_rx: 2,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.01,
        });

        let ml_result = ml_det.detect(&received, &h);
        let sd_result = sd_det.detect(&received, &h);

        for (ml, sd) in ml_result.detected_symbols.iter().zip(sd_result.detected_symbols.iter()) {
            assert!((ml - sd).norm() < 0.01, "ML {:?} != SD {:?}", ml, sd);
        }
    }

    #[test]
    fn test_k_best_detection() {
        let c = ConstellationSet::qpsk();
        let h = ChannelMatrix::new(2, 2, vec![
            Complex64::new(1.0, 0.0), Complex64::new(0.3, 0.2),
            Complex64::new(-0.1, 0.4), Complex64::new(0.9, -0.2),
        ]);

        let tx = vec![c.points[1], c.points[0]];
        let received = h.mat_vec_mul(&tx);

        let mut detector = MimoDetector::new(MimoDetectorConfig {
            algorithm: DetectionAlgorithm::KBest { k: 4 },
            num_tx: 2, num_rx: 2,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.01,
        });

        let result = detector.detect(&received, &h);
        for (det, orig) in result.detected_symbols.iter().zip(tx.iter()) {
            assert!((det - orig).norm() < 0.1);
        }
    }

    #[test]
    fn test_sphere_decoder_reduced_complexity() {
        let c = ConstellationSet::qpsk();
        let mut seed = 12345u64;
        let h_data: Vec<Complex64> = (0..16).map(|_| random_complex(&mut seed) * 0.5).collect();
        let h = ChannelMatrix::new(4, 4, h_data);

        let tx: Vec<Complex64> = (0..4).map(|i| c.points[i % 4]).collect();
        let received = h.mat_vec_mul(&tx);

        let mut detector = MimoDetector::new(MimoDetectorConfig {
            algorithm: DetectionAlgorithm::SphereDecoder { initial_radius: 50.0 },
            num_tx: 4, num_rx: 4,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.01,
        });

        let result = detector.detect(&received, &h);
        // Sphere decoder should visit fewer nodes than exhaustive 4^4 = 256
        assert!(result.nodes_visited < 256,
            "Sphere decoder visited {} nodes, expected < 256", result.nodes_visited);
    }

    #[test]
    fn test_mmse_sic_detection() {
        let c = ConstellationSet::qpsk();
        let h = ChannelMatrix::new(2, 2, vec![
            Complex64::new(0.9, 0.1), Complex64::new(0.2, -0.3),
            Complex64::new(-0.1, 0.5), Complex64::new(0.8, 0.2),
        ]);

        let tx = vec![c.points[0], c.points[3]];
        let received = h.mat_vec_mul(&tx);

        let mut detector = MimoDetector::new(MimoDetectorConfig {
            algorithm: DetectionAlgorithm::MmseSic,
            num_tx: 2, num_rx: 2,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.001,
        });

        let result = detector.detect(&received, &h);
        // At high SNR, MMSE-SIC should find correct symbols
        for (det, orig) in result.detected_symbols.iter().zip(tx.iter()) {
            assert!((det - orig).norm() < 0.3,
                "MMSE-SIC mismatch: {:?} vs {:?}", det, orig);
        }
    }

    #[test]
    fn test_soft_output_sign() {
        let c = ConstellationSet::qpsk();
        let h = ChannelMatrix::identity(2);
        let tx = vec![c.points[0], c.points[0]]; // known symbols
        let received = h.mat_vec_mul(&tx);

        let mut detector = MimoDetector::new(MimoDetectorConfig {
            algorithm: DetectionAlgorithm::MaximumLikelihood,
            num_tx: 2, num_rx: 2,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.01,
        });

        let result = detector.detect(&received, &h);
        // Soft bits should be non-zero at high SNR
        assert!(result.soft_bits.iter().any(|&l| l.abs() > 0.01));
    }

    #[test]
    fn test_ill_conditioned_channel() {
        let h = ChannelMatrix::new(2, 2, vec![
            Complex64::new(1.0, 0.0), Complex64::new(0.999, 0.0),
            Complex64::new(0.0, 1.0), Complex64::new(0.0, 0.999),
        ]);
        let received = vec![Complex64::new(1.0, 1.0), Complex64::new(0.0, 1.0)];

        let mut detector = MimoDetector::new(MimoDetectorConfig {
            algorithm: DetectionAlgorithm::SphereDecoder { initial_radius: 100.0 },
            num_tx: 2, num_rx: 2,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.1,
        });

        let result = detector.detect(&received, &h);
        assert!(result.metric.is_finite());
        assert_eq!(result.detected_symbols.len(), 2);
    }

    #[test]
    fn test_batch_detection_consistency() {
        let c = ConstellationSet::qpsk();
        let h = ChannelMatrix::new(2, 2, vec![
            Complex64::new(0.8, 0.1), Complex64::new(-0.2, 0.5),
            Complex64::new(0.3, -0.4), Complex64::new(0.7, 0.2),
        ]);

        let mut seed = 42u64;
        let vectors: Vec<Vec<Complex64>> = (0..10).map(|_| {
            let tx = vec![
                c.points[(*&mut seed as usize) % 4],
                c.points[(seed.wrapping_mul(7)) as usize % 4],
            ];
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            h.mat_vec_mul(&tx)
        }).collect();

        // Detect individually
        let mut det1 = MimoDetector::new(MimoDetectorConfig {
            algorithm: DetectionAlgorithm::MaximumLikelihood,
            num_tx: 2, num_rx: 2,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.01,
        });
        let individual: Vec<DetectionResult> = vectors.iter()
            .map(|v| det1.detect(v, &h)).collect();

        // Detect in batch
        let mut det2 = MimoDetector::new(MimoDetectorConfig {
            algorithm: DetectionAlgorithm::MaximumLikelihood,
            num_tx: 2, num_rx: 2,
            constellation: ConstellationSet::qpsk(),
            noise_variance: 0.01,
        });
        let batch = det2.detect_batch(&vectors, &h);

        for (ind, bat) in individual.iter().zip(batch.iter()) {
            assert!((ind.metric - bat.metric).abs() < 1e-10);
        }
    }

    #[test]
    fn test_qr_decomposition_accuracy() {
        let mut seed = 99u64;
        let h_data: Vec<Complex64> = (0..16).map(|_| random_complex(&mut seed)).collect();
        let h = ChannelMatrix::new(4, 4, h_data);

        let (q, r) = qr_decompose(&h);

        // Check Q^H * Q ~= I
        let qhq = q.hermitian().mat_mul(&q);
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((qhq.get(i, j).re - expected).abs() < 1e-10,
                    "Q^H*Q[{},{}] = {:?}, expected {}", i, j, qhq.get(i, j), expected);
                if i != j {
                    assert!(qhq.get(i, j).im.abs() < 1e-10);
                }
            }
        }

        // Check H ~= Q * R
        let qr = q.mat_mul(&r);
        let mut err = 0.0;
        let mut norm = 0.0;
        for i in 0..4 {
            for j in 0..4 {
                err += (h.get(i, j) - qr.get(i, j)).norm_sqr();
                norm += h.get(i, j).norm_sqr();
            }
        }
        assert!((err / norm).sqrt() < 1e-12, "Relative error {} too large", (err / norm).sqrt());
    }
}
