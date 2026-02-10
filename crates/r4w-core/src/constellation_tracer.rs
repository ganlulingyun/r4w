//! Constellation Tracer --- real-time constellation diagram generation
//!
//! Provides symbol history tracking, error vector visualization, and
//! modulation quality metrics for received IQ constellations. Includes
//! reference constellations (BPSK through 64-QAM), EVM/MER measurement,
//! nearest-point snapping, and 2-D density histograms for heatmap display.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::constellation_tracer::{
//!     ConstellationTracer, ConstellationRef, ErrorVectorMagnitude,
//! };
//!
//! // Collect received symbols
//! let mut tracer = ConstellationTracer::new(1024);
//! tracer.push((0.98, 0.02));
//! tracer.push((-1.01, -0.03));
//! assert_eq!(tracer.symbols().len(), 2);
//!
//! // Measure EVM against BPSK reference
//! let evm = ErrorVectorMagnitude::new(ConstellationRef::Bpsk);
//! let result = evm.compute(tracer.symbols());
//! assert!(result.evm_rms_percent < 10.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// ConstellationTracer — symbol history ring buffer
// ---------------------------------------------------------------------------

/// Tracks received symbols for constellation diagram rendering.
///
/// Stores up to `max_history` most-recent symbols, silently discarding
/// the oldest when the limit is reached.
#[derive(Debug, Clone)]
pub struct ConstellationTracer {
    symbols: Vec<(f64, f64)>,
    max_history: usize,
}

impl ConstellationTracer {
    /// Create a tracer that retains at most `max_history` symbols.
    pub fn new(max_history: usize) -> Self {
        Self {
            symbols: Vec::with_capacity(max_history.min(4096)),
            max_history,
        }
    }

    /// Append a single received symbol.
    pub fn push(&mut self, symbol: (f64, f64)) {
        if self.symbols.len() >= self.max_history {
            self.symbols.remove(0);
        }
        self.symbols.push(symbol);
    }

    /// Append a batch of received symbols.
    pub fn push_batch(&mut self, symbols: &[(f64, f64)]) {
        for &s in symbols {
            self.push(s);
        }
    }

    /// Current symbol history (oldest first).
    pub fn symbols(&self) -> &[(f64, f64)] {
        &self.symbols
    }

    /// Discard all stored symbols.
    pub fn clear(&mut self) {
        self.symbols.clear();
    }
}

// ---------------------------------------------------------------------------
// ConstellationRef — ideal reference constellations
// ---------------------------------------------------------------------------

/// Ideal reference constellation definitions.
#[derive(Debug, Clone)]
pub enum ConstellationRef {
    /// BPSK: {-1, +1} on the real axis.
    Bpsk,
    /// QPSK: four points at +/-1 +/- j1 (normalized to unit energy).
    Qpsk,
    /// 8-PSK: eight equi-spaced points on the unit circle.
    Psk8,
    /// 16-QAM: 4x4 grid, normalized to unit average power.
    Qam16,
    /// 64-QAM: 8x8 grid, normalized to unit average power.
    Qam64,
    /// User-supplied reference points.
    Custom(Vec<(f64, f64)>),
}

impl ConstellationRef {
    /// Return the ideal constellation points.
    pub fn points(&self) -> Vec<(f64, f64)> {
        match self {
            Self::Bpsk => vec![(-1.0, 0.0), (1.0, 0.0)],

            Self::Qpsk => {
                let s = 1.0_f64 / 2.0_f64.sqrt();
                vec![(s, s), (-s, s), (-s, -s), (s, -s)]
            }

            Self::Psk8 => (0..8)
                .map(|k| {
                    let angle = 2.0 * PI * k as f64 / 8.0;
                    (angle.cos(), angle.sin())
                })
                .collect(),

            Self::Qam16 => {
                let coords = [-3.0_f64, -1.0, 1.0, 3.0];
                // Average power = (2 * (1+9+1+9) + 2*(1+9+1+9)) / 16
                // = (4*(1+1+9+9))/16 * 2 axes => mean |s|^2 = 10
                let scale = 1.0 / 10.0_f64.sqrt();
                let mut pts = Vec::with_capacity(16);
                for &i in &coords {
                    for &q in &coords {
                        pts.push((i * scale, q * scale));
                    }
                }
                pts
            }

            Self::Qam64 => {
                let vals: Vec<f64> = (-3..=3)
                    .step_by(1)
                    .flat_map(|v| {
                        if v >= 0 {
                            Some(2.0 * v as f64 + 1.0)
                        } else {
                            Some(2.0 * v as f64 - 1.0)
                        }
                    })
                    .collect();
                // Use symmetric set: -7, -5, -3, -1, 1, 3, 5, 7
                let coords = [-7.0_f64, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0];
                // mean |s|^2 = 2 * E[x^2] where E[x^2] = (1+9+25+49)*2/8 = 21
                let scale = 1.0 / 42.0_f64.sqrt();
                let mut pts = Vec::with_capacity(64);
                for &i in &coords {
                    for &q in &coords {
                        pts.push((i * scale, q * scale));
                    }
                }
                let _ = vals; // suppress unused warning for intermediate
                pts
            }

            Self::Custom(pts) => pts.clone(),
        }
    }
}

// ---------------------------------------------------------------------------
// nearest_point — closest reference point lookup
// ---------------------------------------------------------------------------

/// Find the reference point closest (Euclidean) to `received`.
///
/// Returns `(index, point)`. Panics if `reference` is empty.
pub fn nearest_point(received: (f64, f64), reference: &[(f64, f64)]) -> (usize, (f64, f64)) {
    assert!(!reference.is_empty(), "reference constellation must not be empty");
    let mut best_idx = 0;
    let mut best_dist = f64::MAX;
    for (i, &(ri, rq)) in reference.iter().enumerate() {
        let d = (received.0 - ri).powi(2) + (received.1 - rq).powi(2);
        if d < best_dist {
            best_dist = d;
            best_idx = i;
        }
    }
    (best_idx, reference[best_idx])
}

// ---------------------------------------------------------------------------
// EVM measurement
// ---------------------------------------------------------------------------

/// Result of an EVM computation over a batch of symbols.
#[derive(Debug, Clone, Copy)]
pub struct EvmResult {
    /// RMS EVM as a percentage (0 = perfect).
    pub evm_rms_percent: f64,
    /// Peak EVM as a percentage.
    pub evm_peak_percent: f64,
    /// SNR estimate derived from EVM (dB). `SNR = -20 log10(EVM_rms)`.
    pub snr_estimate_db: f64,
    /// Mean phase error in degrees.
    pub phase_error_deg: f64,
}

/// Error Vector Magnitude calculator bound to a reference constellation.
#[derive(Debug, Clone)]
pub struct ErrorVectorMagnitude {
    reference: ConstellationRef,
}

impl ErrorVectorMagnitude {
    /// Create an EVM calculator for the given reference.
    pub fn new(reference: ConstellationRef) -> Self {
        Self { reference }
    }

    /// Compute EVM statistics for `received` symbols.
    ///
    /// Each received symbol is snapped to its nearest reference point
    /// before computing the error vector.
    pub fn compute(&self, received: &[(f64, f64)]) -> EvmResult {
        if received.is_empty() {
            return EvmResult {
                evm_rms_percent: 0.0,
                evm_peak_percent: 0.0,
                snr_estimate_db: f64::INFINITY,
                phase_error_deg: 0.0,
            };
        }

        let ref_pts = self.reference.points();
        let mut sum_error_sq = 0.0_f64;
        let mut sum_ref_sq = 0.0_f64;
        let mut peak_evm_sq = 0.0_f64;
        let mut sum_phase_err = 0.0_f64;

        for &(ri, rq) in received {
            let (_idx, (refi, refq)) = nearest_point((ri, rq), &ref_pts);
            let ei = ri - refi;
            let eq = rq - refq;
            let err_sq = ei * ei + eq * eq;
            let ref_sq = refi * refi + refq * refq;

            sum_error_sq += err_sq;
            sum_ref_sq += ref_sq;

            // Per-symbol EVM squared (normalized)
            if ref_sq > 1e-30 {
                let sym_evm_sq = err_sq / ref_sq;
                if sym_evm_sq > peak_evm_sq {
                    peak_evm_sq = sym_evm_sq;
                }
            }

            // Phase error: angle between received and reference vectors
            let ref_phase = refq.atan2(refi);
            let rx_phase = rq.atan2(ri);
            let mut dp = rx_phase - ref_phase;
            // Wrap to [-pi, pi]
            while dp > PI {
                dp -= 2.0 * PI;
            }
            while dp < -PI {
                dp += 2.0 * PI;
            }
            sum_phase_err += dp.abs();
        }

        let n = received.len() as f64;
        let evm_rms = if sum_ref_sq > 1e-30 {
            (sum_error_sq / sum_ref_sq).sqrt()
        } else {
            0.0
        };

        let evm_peak = peak_evm_sq.sqrt();

        let snr_db = if evm_rms > 1e-30 {
            -20.0 * evm_rms.log10()
        } else {
            f64::INFINITY
        };

        let mean_phase_deg = (sum_phase_err / n) * 180.0 / PI;

        EvmResult {
            evm_rms_percent: evm_rms * 100.0,
            evm_peak_percent: evm_peak * 100.0,
            snr_estimate_db: snr_db,
            phase_error_deg: mean_phase_deg,
        }
    }
}

// ---------------------------------------------------------------------------
// ConstellationGrid — 2-D density histogram
// ---------------------------------------------------------------------------

/// 2-D histogram for constellation density / heatmap display.
///
/// The grid covers `[-range, +range]` on both I and Q axes, divided into
/// `resolution x resolution` bins.
#[derive(Debug, Clone)]
pub struct ConstellationGrid {
    resolution: usize,
    range: f64,
    bins: Vec<Vec<u32>>,
}

impl ConstellationGrid {
    /// Create a grid with `resolution x resolution` bins spanning
    /// `[-range, +range]` on each axis.
    pub fn new(resolution: usize, range: f64) -> Self {
        let resolution = resolution.max(1);
        Self {
            resolution,
            range: range.abs().max(1e-12),
            bins: vec![vec![0u32; resolution]; resolution],
        }
    }

    /// Accumulate a batch of symbols into the density grid.
    pub fn accumulate(&mut self, symbols: &[(f64, f64)]) {
        let res = self.resolution as f64;
        for &(i, q) in symbols {
            // Map [-range, +range] -> [0, resolution)
            let ci = ((i + self.range) / (2.0 * self.range) * res).floor() as isize;
            let cq = ((q + self.range) / (2.0 * self.range) * res).floor() as isize;
            if ci >= 0 && ci < self.resolution as isize && cq >= 0 && cq < self.resolution as isize
            {
                self.bins[ci as usize][cq as usize] += 1;
            }
        }
    }

    /// Return the density count for bin `(i, j)`.
    ///
    /// Panics if `i` or `j` is out of range.
    pub fn density(&self, i: usize, j: usize) -> u32 {
        self.bins[i][j]
    }

    /// Export the full grid as a row-major `Vec<Vec<u32>>` (I-outer, Q-inner).
    pub fn to_heatmap(&self) -> Vec<Vec<u32>> {
        self.bins.clone()
    }
}

// ---------------------------------------------------------------------------
// MER helper
// ---------------------------------------------------------------------------

/// Modulation Error Ratio (MER) in dB from EVM expressed as a percentage.
///
/// `MER = -20 * log10(EVM_percent / 100)`
pub fn mer_from_evm(evm_percent: f64) -> f64 {
    if evm_percent <= 0.0 {
        return f64::INFINITY;
    }
    -20.0 * (evm_percent / 100.0).log10()
}

// ---------------------------------------------------------------------------
// Factory helpers
// ---------------------------------------------------------------------------

/// Create a BPSK constellation tracer with 4096-symbol history.
pub fn bpsk_tracer() -> ConstellationTracer {
    ConstellationTracer::new(4096)
}

/// Create a QPSK constellation tracer with 4096-symbol history.
pub fn qpsk_tracer() -> ConstellationTracer {
    ConstellationTracer::new(4096)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_push_and_retrieve_symbols() {
        let mut tracer = ConstellationTracer::new(100);
        tracer.push((1.0, 0.0));
        tracer.push((0.0, 1.0));
        tracer.push_batch(&[(-1.0, 0.0), (0.0, -1.0)]);
        assert_eq!(tracer.symbols().len(), 4);
        assert_eq!(tracer.symbols()[0], (1.0, 0.0));
        assert_eq!(tracer.symbols()[3], (0.0, -1.0));

        tracer.clear();
        assert_eq!(tracer.symbols().len(), 0);
    }

    #[test]
    fn test_max_history_limit_enforced() {
        let mut tracer = ConstellationTracer::new(5);
        for i in 0..10 {
            tracer.push((i as f64, 0.0));
        }
        // Only the last 5 should remain
        assert_eq!(tracer.symbols().len(), 5);
        assert_eq!(tracer.symbols()[0], (5.0, 0.0));
        assert_eq!(tracer.symbols()[4], (9.0, 0.0));
    }

    #[test]
    fn test_bpsk_reference_points() {
        let pts = ConstellationRef::Bpsk.points();
        assert_eq!(pts.len(), 2);
        assert!((pts[0].0 - (-1.0)).abs() < 1e-12);
        assert!((pts[0].1).abs() < 1e-12);
        assert!((pts[1].0 - 1.0).abs() < 1e-12);
        assert!((pts[1].1).abs() < 1e-12);
    }

    #[test]
    fn test_qpsk_reference_points() {
        let pts = ConstellationRef::Qpsk.points();
        assert_eq!(pts.len(), 4);
        let s = 1.0 / 2.0_f64.sqrt();
        // All points should lie at distance 1 from origin
        for &(i, q) in &pts {
            let mag = (i * i + q * q).sqrt();
            assert!((mag - 1.0).abs() < 1e-12, "QPSK point magnitude should be 1.0, got {mag}");
        }
        // Verify the four quadrants are covered
        assert!((pts[0].0 - s).abs() < 1e-12 && (pts[0].1 - s).abs() < 1e-12);
        assert!((pts[1].0 - (-s)).abs() < 1e-12 && (pts[1].1 - s).abs() < 1e-12);
        assert!((pts[2].0 - (-s)).abs() < 1e-12 && (pts[2].1 - (-s)).abs() < 1e-12);
        assert!((pts[3].0 - s).abs() < 1e-12 && (pts[3].1 - (-s)).abs() < 1e-12);
    }

    #[test]
    fn test_qam16_reference_points() {
        let pts = ConstellationRef::Qam16.points();
        assert_eq!(pts.len(), 16);
        // Verify unit average power: mean(|s|^2) ≈ 1.0
        let mean_power: f64 =
            pts.iter().map(|&(i, q)| i * i + q * q).sum::<f64>() / pts.len() as f64;
        assert!(
            (mean_power - 1.0).abs() < 1e-10,
            "16-QAM mean power should be 1.0, got {mean_power}"
        );
    }

    #[test]
    fn test_evm_clean_constellation() {
        // Feed exact QPSK points — EVM should be ~0%
        let ref_pts = ConstellationRef::Qpsk.points();
        let evm = ErrorVectorMagnitude::new(ConstellationRef::Qpsk);
        let result = evm.compute(&ref_pts);
        assert!(
            result.evm_rms_percent < 0.01,
            "Clean EVM should be ~0%, got {}%",
            result.evm_rms_percent
        );
        assert!(
            result.evm_peak_percent < 0.01,
            "Clean peak EVM should be ~0%, got {}%",
            result.evm_peak_percent
        );
    }

    #[test]
    fn test_evm_with_noise() {
        // Add small offsets to BPSK symbols — EVM should be > 0%
        let noisy: Vec<(f64, f64)> = vec![
            (0.9, 0.1),
            (-1.1, -0.05),
            (1.05, -0.08),
            (-0.95, 0.12),
        ];
        let evm = ErrorVectorMagnitude::new(ConstellationRef::Bpsk);
        let result = evm.compute(&noisy);
        assert!(
            result.evm_rms_percent > 1.0,
            "Noisy EVM should be > 1%, got {}%",
            result.evm_rms_percent
        );
        assert!(
            result.snr_estimate_db < 60.0,
            "SNR should be finite, got {} dB",
            result.snr_estimate_db
        );
        assert!(result.phase_error_deg > 0.0, "Phase error should be > 0 deg");
    }

    #[test]
    fn test_nearest_point_selection() {
        let ref_pts = ConstellationRef::Bpsk.points(); // (-1,0) and (1,0)
        // Point close to +1
        let (idx, pt) = nearest_point((0.8, 0.1), &ref_pts);
        assert_eq!(idx, 1);
        assert_eq!(pt, (1.0, 0.0));

        // Point close to -1
        let (idx, pt) = nearest_point((-0.7, -0.2), &ref_pts);
        assert_eq!(idx, 0);
        assert_eq!(pt, (-1.0, 0.0));

        // Equidistant case (origin) — implementation picks first found
        let (idx, _) = nearest_point((0.0, 0.0), &ref_pts);
        assert!(idx == 0 || idx == 1);
    }

    #[test]
    fn test_constellation_grid_histogram() {
        let mut grid = ConstellationGrid::new(10, 2.0);
        // Place symbols at (1.0, 1.0) repeatedly
        let symbols: Vec<(f64, f64)> = vec![(1.0, 1.0); 50];
        grid.accumulate(&symbols);

        // The bin containing (1.0, 1.0) should have count 50
        let heatmap = grid.to_heatmap();
        let total: u32 = heatmap.iter().flat_map(|row| row.iter()).sum();
        assert_eq!(total, 50);

        // The (1.0, 1.0) maps to bin ~7,7 in a 10x10 grid over [-2,2]
        // (1.0 + 2.0) / 4.0 * 10 = 7.5 -> bin 7
        assert_eq!(grid.density(7, 7), 50);
    }

    #[test]
    fn test_mer_from_evm_conversion() {
        // 1% EVM -> MER = -20*log10(0.01) = 40 dB
        let mer = mer_from_evm(1.0);
        assert!(
            (mer - 40.0).abs() < 0.01,
            "1% EVM should give 40 dB MER, got {mer}"
        );

        // 10% EVM -> MER = -20*log10(0.1) = 20 dB
        let mer = mer_from_evm(10.0);
        assert!(
            (mer - 20.0).abs() < 0.01,
            "10% EVM should give 20 dB MER, got {mer}"
        );

        // 100% EVM -> MER = 0 dB
        let mer = mer_from_evm(100.0);
        assert!(
            mer.abs() < 0.01,
            "100% EVM should give 0 dB MER, got {mer}"
        );

        // 0% EVM -> MER = infinity
        let mer = mer_from_evm(0.0);
        assert!(mer.is_infinite() && mer > 0.0);
    }
}
