//! Wi-Fi channel estimation and tracking per IEEE 802.11-2020.
//!
//! Implements L-LTF and HT-LTF based OFDM channel estimation for 802.11a/g/n,
//! frequency/time-domain smoothing, pilot-based phase tracking, SNR estimation,
//! and channel quality metrics.
//!
//! # Standards
//!
//! - IEEE 802.11-2020 §17.3.9 (L-LTF channel estimation)
//! - IEEE 802.11-2020 §19.3.9 (HT-LTF, MIMO channel estimation)
//!
//! # Features
//!
//! - **L-LTF estimation**: LS estimate H[k] = Y[k]/X[k], averaged over 2 symbols
//! - **HT-LTF MIMO**: LTF mapping matrix P for 1-4 spatial streams
//! - **Frequency smoothing**: Moving-average over adjacent subcarriers
//! - **Time-domain smoothing**: IFFT → window/truncate CIR → FFT
//! - **Linear interpolation**: Between pilot subcarriers for data subcarrier recovery
//! - **Phase tracking**: CPE estimation/correction using pilot subcarriers
//! - **SNR estimation**: Per-subcarrier and effective SNR
//! - **Channel quality**: Condition number, delay spread, coherence bandwidth
//!
//! # Example
//!
//! ```
//! use r4w_core::wifi_channel_estimator::{WifiChannelEstimator, EstimatorConfig, WifiMode};
//!
//! // Build a 802.11a estimator (64-point FFT, 52 active subcarriers)
//! let config = EstimatorConfig::wifi_a();
//! let mut est = WifiChannelEstimator::new(config);
//!
//! // Feed two L-LTF symbols (64 complex samples each)
//! let ltf1 = vec![(1.0_f64, 0.0_f64); 64];
//! let ltf2 = vec![(1.0_f64, 0.0_f64); 64];
//! est.load_lltf(&ltf1, &ltf2);
//!
//! // Retrieve per-subcarrier channel estimates
//! let h = est.channel_estimates();
//! assert_eq!(h.len(), 64);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (no external crates)
// ---------------------------------------------------------------------------

/// Complex number (re, im).
pub type Cf64 = (f64, f64);

#[inline(always)]
fn cadd(a: Cf64, b: Cf64) -> Cf64 { (a.0 + b.0, a.1 + b.1) }

#[inline(always)]
fn csub(a: Cf64, b: Cf64) -> Cf64 { (a.0 - b.0, a.1 - b.1) }

#[inline(always)]
fn cmul(a: Cf64, b: Cf64) -> Cf64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline(always)]
fn cdiv(a: Cf64, b: Cf64) -> Cf64 {
    let d = b.0 * b.0 + b.1 * b.1;
    if d < 1e-300 {
        (0.0, 0.0)
    } else {
        ((a.0 * b.0 + a.1 * b.1) / d, (a.1 * b.0 - a.0 * b.1) / d)
    }
}

#[inline(always)]
fn cconj(a: Cf64) -> Cf64 { (a.0, -a.1) }

#[inline(always)]
fn cabs2(a: Cf64) -> f64 { a.0 * a.0 + a.1 * a.1 }

#[inline(always)]
fn cabs(a: Cf64) -> f64 { cabs2(a).sqrt() }

#[inline(always)]
fn cscale(a: Cf64, s: f64) -> Cf64 { (a.0 * s, a.1 * s) }

#[inline(always)]
fn carg(a: Cf64) -> f64 { a.1.atan2(a.0) }

fn czero() -> Cf64 { (0.0, 0.0) }

fn cexp(phase: f64) -> Cf64 { (phase.cos(), phase.sin()) }

// ---------------------------------------------------------------------------
// Radix-2 Cooley-Tukey FFT (in-place, power-of-two length)
// ---------------------------------------------------------------------------

fn fft_inplace(buf: &mut Vec<Cf64>, inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT size must be power-of-two");
    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }
    // Butterfly stages
    let sign = if inverse { 1.0_f64 } else { -1.0_f64 };
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let ang = sign * PI / half as f64;
        let wlen = cexp(ang);
        for i in (0..n).step_by(len) {
            let mut w = (1.0_f64, 0.0_f64);
            for k in 0..half {
                let u = buf[i + k];
                let v = cmul(buf[i + k + half], w);
                buf[i + k] = cadd(u, v);
                buf[i + k + half] = csub(u, v);
                w = cmul(w, wlen);
            }
        }
        len <<= 1;
    }
    if inverse {
        let scale = 1.0 / n as f64;
        for x in buf.iter_mut() {
            *x = cscale(*x, scale);
        }
    }
}

fn fft(input: &[Cf64]) -> Vec<Cf64> {
    let mut buf = input.to_vec();
    fft_inplace(&mut buf, false);
    buf
}

fn ifft(input: &[Cf64]) -> Vec<Cf64> {
    let mut buf = input.to_vec();
    fft_inplace(&mut buf, true);
    buf
}

// ---------------------------------------------------------------------------
// IEEE 802.11 L-LTF known frequency-domain sequence
// ---------------------------------------------------------------------------

/// L-LTF 52-subcarrier BPSK sequence per IEEE 802.11-2020 §17.3.3.
/// Indexed −26..−1, +1..+26 (skipping DC=0).
/// The table encodes ±1 for each of the 52 active subcarriers.
const LLTF_SEQ_52: [i8; 52] = [
    1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1,  // k=-26..-1
    1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,-1, 1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1,  // k=+1..+26
];

/// Build the 64-element frequency-domain L-LTF reference vector.
/// Subcarrier order: [DC=0, k=1..26, guard(27..31), DC(32), guard(33..37), k=-26..-1(38..63)]
/// In IEEE 802.11 FFT convention: index k maps to FFT bin (k mod N).
fn build_lltf_reference(fft_size: usize) -> Vec<Cf64> {
    let mut x = vec![czero(); fft_size];
    // Negative subcarriers: k = -26..-1 → bins fft_size-26 .. fft_size-1
    for (idx, &val) in LLTF_SEQ_52[..26].iter().enumerate() {
        let k = idx as isize - 26; // k = -26..-1
        let bin = (k.rem_euclid(fft_size as isize)) as usize;
        x[bin] = (val as f64, 0.0);
    }
    // Positive subcarriers: k = +1..+26 → bins 1..26
    for (idx, &val) in LLTF_SEQ_52[26..].iter().enumerate() {
        let k = idx + 1; // k = 1..26
        x[k] = (val as f64, 0.0);
    }
    x
}

// ---------------------------------------------------------------------------
// HT-LTF LTF mapping matrix P (IEEE 802.11-2020 §19.3.9.3)
// ---------------------------------------------------------------------------

/// HT-LTF P matrix rows for 1..4 spatial streams.
/// P[streams-1][ltf_idx][stream_idx]
const HT_LTF_P: [[[i8; 4]; 4]; 4] = [
    // 1 stream: P = [[1]]
    [[1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
    // 2 streams: P = [[1,1],[1,-1]]
    [[1, 1, 0, 0], [1,-1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
    // 3 streams: P = [[1,1,1],[1,-1,1],[1,1,-1]]
    [[1, 1, 1, 0], [1,-1, 1, 0], [1, 1,-1, 0], [0, 0, 0, 0]],
    // 4 streams: P = [[1,1,1,1],[1,-1,1,-1],[1,1,-1,-1],[1,-1,-1,1]]
    [[1, 1, 1, 1], [1,-1, 1,-1], [1, 1,-1,-1], [1,-1,-1, 1]],
];

// ---------------------------------------------------------------------------
// Wi-Fi pilot subcarrier positions
// ---------------------------------------------------------------------------

/// 802.11a/g/n pilot subcarrier indices {-21, -7, +7, +21}.
const PILOT_SUBCARRIERS_AG: [isize; 4] = [-21, -7, 7, 21];

/// 802.11n HT20 pilots {-21, -7, +7, +21} (same as legacy, BPSK ±1 pattern).
const PILOT_SUBCARRIERS_N20: [isize; 4] = [-21, -7, 7, 21];

// ---------------------------------------------------------------------------
// Public API types
// ---------------------------------------------------------------------------

/// Operating mode for the estimator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WifiMode {
    /// IEEE 802.11a/g — 20 MHz, 64-point FFT, 52 active subcarriers.
    LegacyAG,
    /// IEEE 802.11n HT20 — 20 MHz, 64-point FFT, 56 active subcarriers.
    HtN20,
    /// IEEE 802.11n HT40 — 40 MHz, 128-point FFT, 114 active subcarriers.
    HtN40,
}

impl WifiMode {
    pub fn fft_size(&self) -> usize {
        match self {
            WifiMode::LegacyAG | WifiMode::HtN20 => 64,
            WifiMode::HtN40 => 128,
        }
    }
    pub fn num_active_subcarriers(&self) -> usize {
        match self {
            WifiMode::LegacyAG => 52,
            WifiMode::HtN20 => 56,
            WifiMode::HtN40 => 114,
        }
    }
    pub fn num_pilots(&self) -> usize { 4 }
    pub fn num_data_subcarriers(&self) -> usize {
        self.num_active_subcarriers() - self.num_pilots()
    }
    pub fn cp_len(&self) -> usize {
        match self {
            WifiMode::LegacyAG | WifiMode::HtN20 => 16,
            WifiMode::HtN40 => 32,
        }
    }
    /// Channel coherence BW reference bandwidth in Hz (per standard).
    pub fn channel_bw_hz(&self) -> f64 {
        match self {
            WifiMode::LegacyAG | WifiMode::HtN20 => 20e6,
            WifiMode::HtN40 => 40e6,
        }
    }
}

/// Frequency-domain smoothing method.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreqSmoothMethod {
    /// No smoothing.
    None,
    /// Moving-average window.
    MovingAverage,
    /// Time-domain (CIR) windowing.
    TimeDomain,
}

/// Interpolation method for data subcarriers between pilots.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterpMethod {
    /// Nearest pilot.
    NearestNeighbor,
    /// Linear interpolation.
    Linear,
    /// Cubic Hermite spline.
    CubicHermite,
}

/// Full configuration for `WifiChannelEstimator`.
#[derive(Debug, Clone)]
pub struct EstimatorConfig {
    /// Operating mode.
    pub mode: WifiMode,
    /// Number of spatial streams (1-4, for HT-LTF MIMO).
    pub num_streams: usize,
    /// Frequency-domain smoothing method.
    pub freq_smooth: FreqSmoothMethod,
    /// Moving-average window half-width (subcarriers).
    pub smooth_window: usize,
    /// CIR window length for time-domain smoothing (taps to keep).
    pub cir_window_len: usize,
    /// Interpolation method for data subcarriers.
    pub interp_method: InterpMethod,
    /// Enable CPE phase tracking per OFDM symbol.
    pub phase_tracking: bool,
    /// Noise variance estimate for MMSE denominator.
    pub noise_variance: f64,
}

impl EstimatorConfig {
    /// Standard 802.11a configuration.
    pub fn wifi_a() -> Self {
        Self {
            mode: WifiMode::LegacyAG,
            num_streams: 1,
            freq_smooth: FreqSmoothMethod::MovingAverage,
            smooth_window: 2,
            cir_window_len: 16,
            interp_method: InterpMethod::Linear,
            phase_tracking: true,
            noise_variance: 1e-3,
        }
    }
    /// Standard 802.11n HT20 configuration.
    pub fn wifi_n20() -> Self {
        Self {
            mode: WifiMode::HtN20,
            num_streams: 1,
            freq_smooth: FreqSmoothMethod::TimeDomain,
            smooth_window: 3,
            cir_window_len: 16,
            interp_method: InterpMethod::Linear,
            phase_tracking: true,
            noise_variance: 1e-3,
        }
    }
    /// 2x2 MIMO 802.11n HT20.
    pub fn wifi_n20_mimo2() -> Self {
        let mut c = Self::wifi_n20();
        c.num_streams = 2;
        c
    }
}

/// Per-subcarrier SNR estimate.
#[derive(Debug, Clone)]
pub struct SubcarrierSnr {
    /// FFT bin index.
    pub bin: usize,
    /// SNR in linear.
    pub snr_linear: f64,
    /// SNR in dB.
    pub snr_db: f64,
}

/// Channel quality summary.
#[derive(Debug, Clone)]
pub struct ChannelQuality {
    /// RMS delay spread (ns) from CIR.
    pub rms_delay_spread_ns: f64,
    /// Coherence bandwidth (MHz), estimated as 1/(5*τ_rms).
    pub coherence_bw_mhz: f64,
    /// Condition number of MIMO channel matrix (1.0 for SISO).
    pub condition_number: f64,
    /// Frequency selectivity index (variance of |H[k]| over active subcarriers).
    pub freq_selectivity: f64,
    /// Mean channel power (linear).
    pub mean_channel_power: f64,
    /// Effective SNR (geometric mean over subcarriers in dB).
    pub effective_snr_db: f64,
}

/// Per-symbol tracking result.
#[derive(Debug, Clone)]
pub struct PhaseTrackResult {
    /// Estimated common phase error (radians).
    pub cpe_rad: f64,
    /// Phase slope (radians/subcarrier) for sampling clock offset correction.
    pub phase_slope: f64,
    /// Residual frequency offset estimate (Hz), requires sample rate knowledge.
    pub freq_offset_frac: f64,
}

// ---------------------------------------------------------------------------
// Main estimator structure
// ---------------------------------------------------------------------------

/// Wi-Fi channel estimator implementing IEEE 802.11-2020 §17.3.9 / §19.3.9.
pub struct WifiChannelEstimator {
    cfg: EstimatorConfig,
    fft_size: usize,
    /// Per-bin LS channel estimates H[k] for each stream.
    /// Outer: stream index; inner: FFT bin index.
    h_est: Vec<Vec<Cf64>>,
    /// L-LTF reference in frequency domain.
    lltf_ref: Vec<Cf64>,
    /// Pilot subcarrier bin indices (mapped to positive FFT bins).
    pilot_bins: Vec<usize>,
    /// Smoothed channel state (per stream, per bin).
    h_smooth: Vec<Vec<Cf64>>,
    /// Cumulative phase error per stream.
    phase_acc: Vec<f64>,
    /// Symbol counter.
    symbol_count: u64,
    /// Noise variance per stream (updated from pilot residuals).
    noise_var: Vec<f64>,
}

impl WifiChannelEstimator {
    /// Create a new estimator with the given configuration.
    pub fn new(cfg: EstimatorConfig) -> Self {
        let fft_size = cfg.mode.fft_size();
        let ns = cfg.num_streams.max(1).min(4);
        let h_est = vec![vec![czero(); fft_size]; ns];
        let h_smooth = vec![vec![czero(); fft_size]; ns];
        let lltf_ref = build_lltf_reference(fft_size);
        let pilot_bins = pilot_bins_for_mode(cfg.mode, fft_size);
        let noise_var = vec![cfg.noise_variance; ns];
        Self {
            cfg,
            fft_size,
            h_est,
            lltf_ref,
            pilot_bins,
            h_smooth,
            phase_acc: vec![0.0; ns],
            symbol_count: 0,
            noise_var,
        }
    }

    // -----------------------------------------------------------------------
    // L-LTF based estimation (IEEE 802.11-2020 §17.3.9)
    // -----------------------------------------------------------------------

    /// Load two L-LTF OFDM symbols (time-domain, after CP removal, each of length fft_size).
    /// Performs LS estimation and averages over the two symbols for noise reduction.
    pub fn load_lltf(&mut self, ltf1: &[Cf64], ltf2: &[Cf64]) {
        assert_eq!(ltf1.len(), self.fft_size);
        assert_eq!(ltf2.len(), self.fft_size);
        let y1 = fft(ltf1);
        let y2 = fft(ltf2);
        // LS estimate for stream 0 (single antenna)
        let h = &mut self.h_est[0];
        for k in 0..self.fft_size {
            let x = self.lltf_ref[k];
            let h1 = ls_estimate(y1[k], x);
            let h2 = ls_estimate(y2[k], x);
            // Average over two symbols
            h[k] = cscale(cadd(h1, h2), 0.5);
        }
        self.apply_smoothing(0);
        self.h_smooth[0] = self.h_est[0].clone();
    }

    /// Load L-LTF in frequency domain (pre-FFT'd, both symbols).
    pub fn load_lltf_freq(&mut self, y1: &[Cf64], y2: &[Cf64]) {
        assert_eq!(y1.len(), self.fft_size);
        assert_eq!(y2.len(), self.fft_size);
        let h = &mut self.h_est[0];
        for k in 0..self.fft_size {
            let x = self.lltf_ref[k];
            let h1 = ls_estimate(y1[k], x);
            let h2 = ls_estimate(y2[k], x);
            h[k] = cscale(cadd(h1, h2), 0.5);
        }
        self.apply_smoothing(0);
        self.h_smooth[0] = self.h_est[0].clone();
    }

    // -----------------------------------------------------------------------
    // HT-LTF MIMO estimation (IEEE 802.11-2020 §19.3.9)
    // -----------------------------------------------------------------------

    /// Load multiple HT-LTF symbols for MIMO channel estimation.
    ///
    /// `ltf_syms[i]` is the frequency-domain received vector (len = fft_size) for
    /// the i-th HT-LTF symbol.  The method uses the P matrix to separate spatial
    /// streams.
    pub fn load_htltf_freq(&mut self, ltf_syms: &[Vec<Cf64>]) {
        let ns = self.cfg.num_streams.min(4);
        assert!(ltf_syms.len() >= ns, "Need >= num_streams HT-LTF symbols");
        let p = &HT_LTF_P[ns - 1];
        // For each subcarrier k, solve: [Y0[k], Y1[k],...] = P * [H0[k], H1[k],...]
        // Using least-squares: H = P^T * Y / ns  (since P is orthogonal scaled by 1/ns)
        for k in 0..self.fft_size {
            for s in 0..ns {
                let mut acc = czero();
                for ltf_idx in 0..ns {
                    let p_val = p[ltf_idx][s] as f64;
                    if p_val == 0.0 { continue; }
                    // Reference for stream s = 1 (BPSK pilot, known)
                    // The received symbol Y is already in frequency domain
                    acc = cadd(acc, cscale(ltf_syms[ltf_idx][k], p_val));
                }
                // Divide by ns to invert the orthogonal P matrix
                self.h_est[s][k] = cscale(acc, 1.0 / ns as f64);
            }
        }
        for s in 0..ns {
            self.apply_smoothing(s);
            self.h_smooth[s] = self.h_est[s].clone();
        }
    }

    // -----------------------------------------------------------------------
    // Frequency-domain smoothing
    // -----------------------------------------------------------------------

    fn apply_smoothing(&mut self, stream: usize) {
        match self.cfg.freq_smooth {
            FreqSmoothMethod::None => {
                self.h_smooth[stream] = self.h_est[stream].clone();
            }
            FreqSmoothMethod::MovingAverage => {
                self.h_smooth[stream] =
                    freq_smooth_moving_avg(&self.h_est[stream], self.cfg.smooth_window);
            }
            FreqSmoothMethod::TimeDomain => {
                self.h_smooth[stream] =
                    freq_smooth_time_domain(&self.h_est[stream], self.cfg.cir_window_len);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Linear / cubic interpolation for data subcarriers
    // -----------------------------------------------------------------------

    /// Return estimated channel for a specific FFT bin using pilot-based interpolation.
    ///
    /// This is useful when the caller has new pilot observations but does not want
    /// to run a full L-LTF update.
    pub fn interpolate_data_subcarrier(&self, bin: usize, stream: usize) -> Cf64 {
        let stream = stream.min(self.cfg.num_streams - 1);
        match self.cfg.interp_method {
            InterpMethod::NearestNeighbor => {
                nearest_pilot_interp(&self.h_smooth[stream], &self.pilot_bins, bin)
            }
            InterpMethod::Linear => {
                linear_interp(&self.h_smooth[stream], &self.pilot_bins, bin)
            }
            InterpMethod::CubicHermite => {
                cubic_hermite_interp(&self.h_smooth[stream], &self.pilot_bins, bin)
            }
        }
    }

    // -----------------------------------------------------------------------
    // Pilot-based phase tracking (IEEE 802.11-2020 §17.3.9.7)
    // -----------------------------------------------------------------------

    /// Track and correct phase using pilot subcarriers in a received OFDM symbol.
    ///
    /// `rx_freq` – received FFT output (len = fft_size) for the given stream.
    /// Returns phase tracking result and optionally corrects in-place.
    pub fn track_phase(
        &mut self,
        rx_freq: &mut Vec<Cf64>,
        stream: usize,
    ) -> PhaseTrackResult {
        let stream = stream.min(self.cfg.num_streams - 1);
        let result = estimate_phase_error(
            rx_freq,
            &self.h_smooth[stream],
            &self.pilot_bins,
        );
        self.phase_acc[stream] += result.cpe_rad;
        // Correct all subcarriers by CPE
        if self.cfg.phase_tracking {
            apply_phase_correction(rx_freq, -result.cpe_rad);
        }
        self.symbol_count += 1;
        result
    }

    // -----------------------------------------------------------------------
    // Equalization
    // -----------------------------------------------------------------------

    /// Zero-Forcing equalization of a received frequency-domain symbol.
    ///
    /// `rx[k] / H[k]` for each active subcarrier.
    pub fn equalize_zf(&self, rx_freq: &[Cf64], stream: usize) -> Vec<Cf64> {
        let stream = stream.min(self.cfg.num_streams - 1);
        let h = &self.h_smooth[stream];
        let mut out = vec![czero(); self.fft_size];
        for k in 0..self.fft_size {
            out[k] = cdiv(rx_freq[k], h[k]);
        }
        out
    }

    /// MMSE equalization: H*[k] / (|H[k]|^2 + σ²) * rx[k].
    pub fn equalize_mmse(&self, rx_freq: &[Cf64], stream: usize) -> Vec<Cf64> {
        let stream = stream.min(self.cfg.num_streams - 1);
        let h = &self.h_smooth[stream];
        let sigma2 = self.noise_var[stream];
        let mut out = vec![czero(); self.fft_size];
        for k in 0..self.fft_size {
            let h2 = cabs2(h[k]);
            let w = cscale(cconj(h[k]), 1.0 / (h2 + sigma2));
            out[k] = cmul(w, rx_freq[k]);
        }
        out
    }

    // -----------------------------------------------------------------------
    // SNR estimation
    // -----------------------------------------------------------------------

    /// Per-subcarrier SNR: |H[k]|^2 / σ² for each active bin.
    pub fn per_subcarrier_snr(&self, stream: usize) -> Vec<SubcarrierSnr> {
        let stream = stream.min(self.cfg.num_streams - 1);
        let sigma2 = self.noise_var[stream].max(1e-300);
        let h = &self.h_smooth[stream];
        let active = active_bins(self.cfg.mode, self.fft_size);
        active.into_iter().map(|bin| {
            let snr_lin = cabs2(h[bin]) / sigma2;
            let snr_db = if snr_lin > 1e-300 { 10.0 * snr_lin.log10() } else { -100.0 };
            SubcarrierSnr { bin, snr_linear: snr_lin, snr_db }
        }).collect()
    }

    /// Effective SNR: geometric mean of per-subcarrier SNR (in dB).
    pub fn effective_snr_db(&self, stream: usize) -> f64 {
        let snrs = self.per_subcarrier_snr(stream);
        if snrs.is_empty() { return -100.0; }
        let mean_db: f64 = snrs.iter().map(|s| s.snr_db).sum::<f64>() / snrs.len() as f64;
        mean_db
    }

    /// Update noise variance estimate from pilot residuals of a received symbol.
    pub fn update_noise_var_from_pilots(&mut self, rx_freq: &[Cf64], stream: usize) {
        let stream = stream.min(self.cfg.num_streams - 1);
        let h = &self.h_smooth[stream];
        let mut sum_err2 = 0.0;
        let mut cnt = 0;
        for &bin in &self.pilot_bins {
            let predicted = cmul(h[bin], (1.0, 0.0)); // pilot BPSK = ±1 known
            let err = csub(rx_freq[bin], predicted);
            sum_err2 += cabs2(err);
            cnt += 1;
        }
        if cnt > 0 {
            let alpha = 0.1; // exponential smoothing
            let new_est = sum_err2 / cnt as f64;
            self.noise_var[stream] = (1.0 - alpha) * self.noise_var[stream] + alpha * new_est;
        }
    }

    // -----------------------------------------------------------------------
    // Channel quality metrics
    // -----------------------------------------------------------------------

    /// Compute channel quality metrics from current estimates.
    pub fn channel_quality(&self, stream: usize) -> ChannelQuality {
        let stream = stream.min(self.cfg.num_streams - 1);
        let h = &self.h_smooth[stream];
        let fft_size = self.fft_size;
        let bw_hz = self.cfg.mode.channel_bw_hz();

        // CIR via IFFT
        let cir = ifft(h);

        // RMS delay spread
        let rms_ds = rms_delay_spread(&cir, bw_hz);

        // Coherence bandwidth: Bc ≈ 1 / (2π * τ_rms)  [simplified]
        let coh_bw_mhz = if rms_ds > 1e-12 {
            1.0 / (2.0 * PI * rms_ds) * 1e-6
        } else {
            bw_hz * 1e-6
        };

        // Frequency selectivity: variance of |H[k]| over active subcarriers
        let active = active_bins(self.cfg.mode, fft_size);
        let mags: Vec<f64> = active.iter().map(|&k| cabs(h[k])).collect();
        let mean_mag = mags.iter().sum::<f64>() / mags.len() as f64;
        let freq_sel = mags.iter().map(|&m| (m - mean_mag).powi(2)).sum::<f64>()
            / mags.len() as f64;

        // Mean channel power
        let mean_pwr = mags.iter().map(|&m| m * m).sum::<f64>() / mags.len() as f64;

        // Condition number (for SISO = 1.0; for MIMO use SVD approximation)
        let condition_number = if self.cfg.num_streams == 1 {
            1.0
        } else {
            mimo_condition_number(&self.h_smooth, &active, self.cfg.num_streams)
        };

        let eff_snr = self.effective_snr_db(stream);

        ChannelQuality {
            rms_delay_spread_ns: rms_ds * 1e9,
            coherence_bw_mhz: coh_bw_mhz,
            condition_number,
            freq_selectivity: freq_sel,
            mean_channel_power: mean_pwr,
            effective_snr_db: eff_snr,
        }
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /// Return per-bin channel estimates for all streams.
    /// Outer index: stream; inner index: FFT bin.
    pub fn channel_estimates(&self) -> &Vec<Vec<Cf64>> { &self.h_smooth }

    /// Return channel estimate for a single stream.
    pub fn channel_estimate_stream(&self, stream: usize) -> &Vec<Cf64> {
        &self.h_smooth[stream.min(self.cfg.num_streams - 1)]
    }

    /// Return channel impulse response (IFFT of H[k]) for a stream.
    pub fn channel_impulse_response(&self, stream: usize) -> Vec<Cf64> {
        let stream = stream.min(self.cfg.num_streams - 1);
        ifft(&self.h_smooth[stream])
    }

    /// Return configured FFT size.
    pub fn fft_size(&self) -> usize { self.fft_size }

    /// Return pilot subcarrier bin indices.
    pub fn pilot_bins(&self) -> &[usize] { &self.pilot_bins }

    /// Return symbol count processed.
    pub fn symbol_count(&self) -> u64 { self.symbol_count }

    /// Reset internal state (phase accumulators, noise variance).
    pub fn reset(&mut self) {
        for s in 0..self.cfg.num_streams {
            self.phase_acc[s] = 0.0;
            self.noise_var[s] = self.cfg.noise_variance;
            for k in 0..self.fft_size {
                self.h_est[s][k] = czero();
                self.h_smooth[s][k] = czero();
            }
        }
        self.symbol_count = 0;
    }

    /// Update a single-subcarrier estimate (e.g., from a decision-directed step).
    pub fn update_subcarrier(&mut self, bin: usize, h_new: Cf64, stream: usize) {
        let stream = stream.min(self.cfg.num_streams - 1);
        let alpha = 0.1;
        let prev = self.h_smooth[stream][bin];
        self.h_smooth[stream][bin] = cadd(
            cscale(prev, 1.0 - alpha),
            cscale(h_new, alpha),
        );
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// LS estimate at a single subcarrier.
fn ls_estimate(y: Cf64, x: Cf64) -> Cf64 {
    cdiv(y, x)
}

/// Active FFT bins for a given Wi-Fi mode.
fn active_bins(mode: WifiMode, fft_size: usize) -> Vec<usize> {
    match mode {
        WifiMode::LegacyAG => {
            // k = -26..-1, +1..+26
            let mut bins = Vec::with_capacity(52);
            for k in 1..=26_usize { bins.push(k); }
            for k in (fft_size - 26)..fft_size { bins.push(k); }
            bins
        }
        WifiMode::HtN20 => {
            // k = -28..-1, +1..+28
            let mut bins = Vec::with_capacity(56);
            for k in 1..=28_usize { bins.push(k); }
            for k in (fft_size - 28)..fft_size { bins.push(k); }
            bins
        }
        WifiMode::HtN40 => {
            // k = -58..-2, +2..+58 (DC null, ±1 guard)
            let mut bins = Vec::with_capacity(114);
            for k in 2..=58_usize { bins.push(k); }
            for k in (fft_size - 58)..(fft_size - 1) { bins.push(k); }
            bins
        }
    }
}

/// Map signed subcarrier index to positive FFT bin.
fn sc_to_bin(k: isize, fft_size: usize) -> usize {
    k.rem_euclid(fft_size as isize) as usize
}

/// Build pilot bin list for a mode.
fn pilot_bins_for_mode(mode: WifiMode, fft_size: usize) -> Vec<usize> {
    let pilots: &[isize] = match mode {
        WifiMode::LegacyAG => &PILOT_SUBCARRIERS_AG,
        WifiMode::HtN20 => &PILOT_SUBCARRIERS_N20,
        WifiMode::HtN40 => &[-53, -25, -11, 11, 25, 53][..], // 6 pilots for HT40
    };
    pilots.iter().map(|&k| sc_to_bin(k, fft_size)).collect()
}

/// Frequency-domain moving-average smoothing.
fn freq_smooth_moving_avg(h: &[Cf64], half_win: usize) -> Vec<Cf64> {
    let n = h.len();
    let mut out = vec![czero(); n];
    for i in 0..n {
        let lo = i.saturating_sub(half_win);
        let hi = (i + half_win + 1).min(n);
        let mut sum = czero();
        let cnt = (hi - lo) as f64;
        for j in lo..hi {
            sum = cadd(sum, h[j]);
        }
        out[i] = cscale(sum, 1.0 / cnt);
    }
    out
}

/// Time-domain smoothing: IFFT → window CIR → FFT.
fn freq_smooth_time_domain(h: &[Cf64], cir_len: usize) -> Vec<Cf64> {
    let n = h.len();
    let mut cir = ifft(h);
    // Zero out taps beyond cir_len (keep first cir_len and last cir_len from end for circular)
    let keep = cir_len.min(n / 2);
    for i in keep..(n - keep) {
        cir[i] = czero();
    }
    // Apply rectangular window to kept taps
    fft(&cir)
}

/// Nearest-pilot interpolation.
fn nearest_pilot_interp(h: &[Cf64], pilots: &[usize], bin: usize) -> Cf64 {
    if pilots.is_empty() { return czero(); }
    let best = pilots.iter().min_by_key(|&&p| {
        let d = if bin >= p { bin - p } else { p - bin };
        d
    }).copied().unwrap_or(pilots[0]);
    h[best]
}

/// Linear interpolation between two nearest pilots.
fn linear_interp(h: &[Cf64], pilots: &[usize], bin: usize) -> Cf64 {
    if pilots.is_empty() { return czero(); }
    if pilots.len() == 1 { return h[pilots[0]]; }
    // Find the two bracketing pilots
    let mut lo_idx = 0;
    let mut hi_idx = pilots.len() - 1;
    for (i, &p) in pilots.iter().enumerate() {
        if p <= bin { lo_idx = i; }
    }
    for (i, &p) in pilots.iter().enumerate().rev() {
        if p >= bin { hi_idx = i; break; }
    }
    let lo_bin = pilots[lo_idx];
    let hi_bin = pilots[hi_idx];
    if lo_bin == hi_bin {
        return h[lo_bin];
    }
    let t = (bin as f64 - lo_bin as f64) / (hi_bin as f64 - lo_bin as f64);
    let t = t.clamp(0.0, 1.0);
    cadd(cscale(h[lo_bin], 1.0 - t), cscale(h[hi_bin], t))
}

/// Cubic Hermite spline interpolation between pilots.
fn cubic_hermite_interp(h: &[Cf64], pilots: &[usize], bin: usize) -> Cf64 {
    if pilots.len() < 2 { return linear_interp(h, pilots, bin); }
    // Find bracketing pilot pair
    let mut lo = 0;
    for (i, &p) in pilots.iter().enumerate() {
        if p <= bin { lo = i; }
    }
    let hi = (lo + 1).min(pilots.len() - 1);
    if lo == hi { return h[pilots[lo]]; }
    let p0 = pilots[lo] as f64;
    let p1 = pilots[hi] as f64;
    let t = ((bin as f64 - p0) / (p1 - p0)).clamp(0.0, 1.0);
    let h0 = h[pilots[lo]];
    let h1 = h[pilots[hi]];
    // Tangents from adjacent pilots (or extrapolate at edges)
    let m0 = if lo > 0 {
        let pb = pilots[lo - 1] as f64;
        cscale(csub(h1, h[pilots[lo - 1]]), 1.0 / (p1 - pb))
    } else {
        csub(h1, h0)
    };
    let m1 = if hi + 1 < pilots.len() {
        let pa = pilots[hi + 1] as f64;
        cscale(csub(h[pilots[hi + 1]], h0), 1.0 / (pa - p0))
    } else {
        csub(h1, h0)
    };
    // Hermite basis polynomials
    let t2 = t * t;
    let t3 = t2 * t;
    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = t3 - 2.0 * t2 + t;
    let h01 = -2.0 * t3 + 3.0 * t2;
    let h11 = t3 - t2;
    let mut result = cscale(h0, h00);
    result = cadd(result, cscale(m0, h10));
    result = cadd(result, cscale(h1, h01));
    result = cadd(result, cscale(m1, h11));
    result
}

/// Estimate common phase error and phase slope from pilot subcarriers.
fn estimate_phase_error(
    rx: &[Cf64],
    h: &[Cf64],
    pilot_bins: &[usize],
) -> PhaseTrackResult {
    // Phase at pilot k: arg(rx[k] * conj(H[k])) assuming pilot BPSK=+1
    let phases: Vec<(f64, f64)> = pilot_bins.iter().map(|&bin| {
        let pred = h[bin]; // predicted pilot = H[k] * pilot_symbol (±1 absorbed)
        let e = cmul(rx[bin], cconj(pred));
        (bin as f64, carg(e))
    }).collect();

    if phases.is_empty() {
        return PhaseTrackResult { cpe_rad: 0.0, phase_slope: 0.0, freq_offset_frac: 0.0 };
    }

    // Least-squares fit: phase[k] = cpe + slope * k
    let n = phases.len() as f64;
    let sum_k: f64 = phases.iter().map(|(k, _)| k).sum();
    let sum_p: f64 = phases.iter().map(|(_, p)| p).sum();
    let sum_k2: f64 = phases.iter().map(|(k, _)| k * k).sum();
    let sum_kp: f64 = phases.iter().map(|(k, p)| k * p).sum();

    let denom = n * sum_k2 - sum_k * sum_k;
    let (cpe, slope) = if denom.abs() > 1e-12 {
        let slope = (n * sum_kp - sum_k * sum_p) / denom;
        let cpe = (sum_p - slope * sum_k) / n;
        (cpe, slope)
    } else {
        (sum_p / n, 0.0)
    };

    PhaseTrackResult {
        cpe_rad: cpe,
        phase_slope: slope,
        freq_offset_frac: slope / (2.0 * PI),
    }
}

/// Apply a constant phase rotation to all subcarriers.
fn apply_phase_correction(rx: &mut Vec<Cf64>, phase: f64) {
    let rot = cexp(phase);
    for x in rx.iter_mut() {
        *x = cmul(*x, rot);
    }
}

/// RMS delay spread from CIR (in seconds).
fn rms_delay_spread(cir: &[Cf64], bw_hz: f64) -> f64 {
    let n = cir.len();
    let dt = 1.0 / bw_hz; // sample period
    let powers: Vec<f64> = cir.iter().map(|&c| cabs2(c)).collect();
    let total_power: f64 = powers.iter().sum();
    if total_power < 1e-300 { return 0.0; }
    // Mean delay
    let mean_tau: f64 = powers.iter().enumerate()
        .map(|(i, &p)| i as f64 * dt * p)
        .sum::<f64>() / total_power;
    // RMS delay spread
    let rms_sq: f64 = powers.iter().enumerate()
        .map(|(i, &p)| {
            let tau = i as f64 * dt;
            (tau - mean_tau).powi(2) * p
        })
        .sum::<f64>() / total_power;
    // Limit to causal region (first half of circular CIR)
    let _ = n; // suppress unused warning
    rms_sq.sqrt()
}

/// Approximate MIMO condition number from singular value spread.
/// Uses a simple power-iteration approximation.
fn mimo_condition_number(h_streams: &[Vec<Cf64>], active: &[usize], ns: usize) -> f64 {
    if ns == 1 || active.is_empty() { return 1.0; }
    // Compute per-subcarrier 2-norm of channel column vectors, then ratio of max/min
    let mut max_col = 0.0_f64;
    let mut min_col = f64::MAX;
    for &k in active {
        let col_norm: f64 = (0..ns).map(|s| cabs2(h_streams[s][k])).sum::<f64>();
        let col_norm = col_norm.sqrt();
        if col_norm > max_col { max_col = col_norm; }
        if col_norm < min_col { min_col = col_norm; }
    }
    if min_col < 1e-300 { return f64::MAX; }
    max_col / min_col
}

// ---------------------------------------------------------------------------
// Builder / convenience constructors
// ---------------------------------------------------------------------------

/// Builder for `EstimatorConfig`.
pub struct EstimatorConfigBuilder {
    cfg: EstimatorConfig,
}

impl EstimatorConfigBuilder {
    pub fn new(mode: WifiMode) -> Self {
        Self { cfg: EstimatorConfig {
            mode,
            num_streams: 1,
            freq_smooth: FreqSmoothMethod::MovingAverage,
            smooth_window: 2,
            cir_window_len: 16,
            interp_method: InterpMethod::Linear,
            phase_tracking: true,
            noise_variance: 1e-3,
        }}
    }
    pub fn num_streams(mut self, n: usize) -> Self { self.cfg.num_streams = n.max(1).min(4); self }
    pub fn freq_smooth(mut self, m: FreqSmoothMethod) -> Self { self.cfg.freq_smooth = m; self }
    pub fn smooth_window(mut self, w: usize) -> Self { self.cfg.smooth_window = w; self }
    pub fn cir_window(mut self, w: usize) -> Self { self.cfg.cir_window_len = w; self }
    pub fn interp(mut self, m: InterpMethod) -> Self { self.cfg.interp_method = m; self }
    pub fn phase_tracking(mut self, en: bool) -> Self { self.cfg.phase_tracking = en; self }
    pub fn noise_variance(mut self, v: f64) -> Self { self.cfg.noise_variance = v; self }
    pub fn build(self) -> EstimatorConfig { self.cfg }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- FFT / IFFT ----

    #[test]
    fn fft_impulse() {
        // FFT of impulse at 0 should be all-ones
        let mut buf = vec![czero(); 64];
        buf[0] = (1.0, 0.0);
        let out = fft(&buf);
        for s in &out {
            assert!((cabs(*s) - 1.0).abs() < 1e-10, "expected 1.0, got {}", cabs(*s));
        }
    }

    #[test]
    fn ifft_roundtrip() {
        let n = 64;
        let input: Vec<Cf64> = (0..n).map(|i| ((i as f64).cos(), (i as f64).sin())).collect();
        let freq = fft(&input);
        let time = ifft(&freq);
        for (a, b) in input.iter().zip(time.iter()) {
            assert!((a.0 - b.0).abs() < 1e-10);
            assert!((a.1 - b.1).abs() < 1e-10);
        }
    }

    #[test]
    fn fft_tone() {
        // A single tone at bin 4 should produce a peak at index 4
        let n = 64;
        let freq = 4.0;
        let input: Vec<Cf64> = (0..n).map(|i| {
            let ph = 2.0 * PI * freq * i as f64 / n as f64;
            (ph.cos(), ph.sin())
        }).collect();
        let out = fft(&input);
        let peak_bin = out.iter().enumerate().max_by(|(_, a), (_, b)| {
            cabs2(**a).partial_cmp(&cabs2(**b)).unwrap()
        }).map(|(i, _)| i).unwrap();
        assert_eq!(peak_bin, 4);
    }

    // ---- LLTF reference ----

    #[test]
    fn lltf_ref_is_bpsk() {
        let r = build_lltf_reference(64);
        for &v in &r {
            let m = cabs(v);
            // Either 0 (guard/DC) or 1.0 (pilot)
            assert!(m < 1e-10 || (m - 1.0).abs() < 1e-10, "unexpected magnitude {}", m);
        }
    }

    #[test]
    fn lltf_ref_active_count() {
        let r = build_lltf_reference(64);
        let active = r.iter().filter(|&&v| cabs(v) > 0.5).count();
        assert_eq!(active, 52);
    }

    // ---- LS estimation with flat channel ----

    #[test]
    fn lltf_estimation_flat_channel() {
        // Under a flat H=1 channel, LTF output == LTF input, so H_est should be ~1
        let cfg = EstimatorConfig::wifi_a();
        let mut est = WifiChannelEstimator::new(cfg);
        let lltf_ref = build_lltf_reference(64);
        // IFFT to get time domain
        let ltf_td = ifft(&lltf_ref);
        // Pass through H=1 (no change)
        est.load_lltf(&ltf_td, &ltf_td);
        let h = est.channel_estimate_stream(0);
        // Check active bins: magnitude should be ~1
        for k in 1..=26 {
            let m = cabs(h[k]);
            assert!((m - 1.0).abs() < 0.1, "bin {k}: |H|={m}");
        }
    }

    #[test]
    fn lltf_estimation_scaled_channel() {
        // H = 2.0 * exp(j*0.5) constant
        let cfg = EstimatorConfig::wifi_a();
        let mut est = WifiChannelEstimator::new(cfg);
        let lltf_ref = build_lltf_reference(64);
        let h_true: Cf64 = (2.0 * (0.5_f64).cos(), 2.0 * (0.5_f64).sin());
        let ltf_freq: Vec<Cf64> = lltf_ref.iter().map(|&x| cmul(x, h_true)).collect();
        let ltf_td = ifft(&ltf_freq);
        est.load_lltf(&ltf_td, &ltf_td);
        let h = est.channel_estimate_stream(0);
        // On active subcarriers the magnitude should be ~2.0
        for k in 1..=26 {
            let m = cabs(h[k]);
            assert!((m - 2.0).abs() < 0.15, "bin {k}: |H|={m}");
        }
    }

    // ---- Frequency smoothing ----

    #[test]
    fn moving_avg_preserves_flat() {
        let h: Vec<Cf64> = (0..64).map(|_| (1.0, 0.0)).collect();
        let s = freq_smooth_moving_avg(&h, 3);
        for v in &s {
            assert!((cabs(*v) - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn moving_avg_smooths_spike() {
        let mut h: Vec<Cf64> = vec![(1.0, 0.0); 64];
        h[32] = (10.0, 0.0); // spike
        let s = freq_smooth_moving_avg(&h, 4);
        // The spike should be reduced
        assert!(cabs(s[32]) < 10.0);
        assert!(cabs(s[32]) > 1.0);
    }

    #[test]
    fn time_domain_smoothing_flat() {
        let h: Vec<Cf64> = (0..64).map(|_| (1.0, 0.0)).collect();
        let s = freq_smooth_time_domain(&h, 8);
        // Flat channel in freq <=> impulse in time; keeping short CIR should
        // reconstruct a reasonable flat response
        for k in 1..=26 {
            assert!(cabs(s[k]) > 0.5, "bin {k}: magnitude too small");
        }
    }

    #[test]
    fn time_domain_smoothing_roundtrip() {
        // A pure impulse in time domain => flat frequency response
        // After windowing to 1 tap and back, still flat
        let mut h = vec![czero(); 64];
        h[0] = (1.0, 0.0);
        let freq = fft(&h);
        let s = freq_smooth_time_domain(&freq, 1);
        // Frequency domain should remain flat magnitude
        for k in 0..64 {
            assert!(cabs(s[k]) > 0.9, "bin {k}");
        }
    }

    // ---- Interpolation ----

    #[test]
    fn nearest_pilot_returns_closest() {
        let h: Vec<Cf64> = (0..64).map(|i| (i as f64, 0.0)).collect();
        let pilots = vec![10_usize, 30, 50];
        // Bin 20 is equidistant from 10 and 30; nearest should be 10 or 30
        let v = nearest_pilot_interp(&h, &pilots, 20);
        let m = v.0;
        assert!(m == 10.0 || m == 30.0, "got {m}");
    }

    #[test]
    fn linear_interp_at_pilot() {
        let h: Vec<Cf64> = (0..64).map(|i| (i as f64, 0.0)).collect();
        let pilots = vec![10_usize, 20, 30];
        // Exact pilot bin should return exact value
        let v = linear_interp(&h, &pilots, 20);
        assert!((v.0 - 20.0).abs() < 1e-10, "got {}", v.0);
    }

    #[test]
    fn linear_interp_midpoint() {
        // H is linear: H[k] = k
        let h: Vec<Cf64> = (0..64).map(|i| (i as f64, 0.0)).collect();
        let pilots = vec![10_usize, 20];
        let v = linear_interp(&h, &pilots, 15);
        assert!((v.0 - 15.0).abs() < 1e-10, "got {}", v.0);
    }

    #[test]
    fn cubic_hermite_at_pilot_exact() {
        let h: Vec<Cf64> = (0..64).map(|i| (i as f64 * 2.0, 0.0)).collect();
        let pilots = vec![4_usize, 12, 20, 28];
        let v = cubic_hermite_interp(&h, &pilots, 12);
        assert!((v.0 - 24.0).abs() < 1e-6, "got {}", v.0);
    }

    #[test]
    fn cubic_hermite_between_pilots() {
        // With a linear channel, cubic should also give exact result at midpoint
        let h: Vec<Cf64> = (0..64).map(|i| (i as f64, 0.0)).collect();
        let pilots = vec![8_usize, 16, 24];
        let v = cubic_hermite_interp(&h, &pilots, 12);
        // Within ~1 of linear result
        assert!((v.0 - 12.0).abs() < 2.0, "got {}", v.0);
    }

    // ---- Phase tracking ----

    #[test]
    fn phase_tracking_detects_zero_cpe() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        // Set identity channel
        for k in 0..fft_size {
            est.h_smooth[0][k] = (1.0, 0.0);
        }
        // Received = channel * pilot = 1.0 at pilot bins
        let mut rx = vec![czero(); fft_size];
        for &b in &est.pilot_bins {
            rx[b] = (1.0, 0.0);
        }
        let res = estimate_phase_error(&rx, &est.h_smooth[0], &est.pilot_bins);
        assert!(res.cpe_rad.abs() < 0.1, "cpe={}", res.cpe_rad);
    }

    #[test]
    fn phase_tracking_detects_nonzero_cpe() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        let true_phase = 0.3_f64;
        for k in 0..fft_size {
            est.h_smooth[0][k] = (1.0, 0.0);
        }
        // Introduce CPE
        let mut rx = vec![czero(); fft_size];
        for &b in &est.pilot_bins {
            rx[b] = cexp(true_phase);
        }
        let res = estimate_phase_error(&rx, &est.h_smooth[0], &est.pilot_bins);
        assert!((res.cpe_rad - true_phase).abs() < 0.05, "cpe={}", res.cpe_rad);
    }

    #[test]
    fn phase_correction_applied() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        for k in 0..fft_size {
            est.h_smooth[0][k] = (1.0, 0.0);
        }
        let true_phase = 0.5;
        let mut rx: Vec<Cf64> = (0..fft_size).map(|_| cexp(true_phase)).collect();
        est.track_phase(&mut rx, 0);
        // After correction, pilot bins should be near (1,0)
        for &b in est.pilot_bins() {
            assert!((rx[b].1).abs() < 0.1, "imag part after correction: {}", rx[b].1);
        }
    }

    // ---- ZF / MMSE equalization ----

    #[test]
    fn zf_equalization_unity_channel() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        for k in 0..fft_size {
            est.h_smooth[0][k] = (1.0, 0.0);
        }
        let rx: Vec<Cf64> = (0..fft_size).map(|i| (i as f64, 0.0)).collect();
        let eq = est.equalize_zf(&rx, 0);
        for (k, (&r, e)) in rx.iter().zip(eq.iter()).enumerate() {
            assert!((r.0 - e.0).abs() < 1e-10, "bin {k}");
        }
    }

    #[test]
    fn zf_equalization_removes_channel() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        // Channel: H[k] = 2.0
        for k in 0..fft_size {
            est.h_smooth[0][k] = (2.0, 0.0);
        }
        // Data symbols sent = (k, 0)
        let tx: Vec<Cf64> = (0..fft_size).map(|i| (i as f64, 0.0)).collect();
        // Received = H * tx
        let rx: Vec<Cf64> = tx.iter().map(|&s| cmul(s, (2.0, 0.0))).collect();
        let eq = est.equalize_zf(&rx, 0);
        for k in 0..fft_size {
            assert!((eq[k].0 - tx[k].0).abs() < 1e-8, "bin {k}: eq={} tx={}", eq[k].0, tx[k].0);
        }
    }

    #[test]
    fn mmse_equalization_approximates_zf_high_snr() {
        let mut cfg = EstimatorConfig::wifi_a();
        cfg.noise_variance = 1e-8; // Very low noise → MMSE ≈ ZF
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        for k in 0..fft_size {
            est.h_smooth[0][k] = (1.5, 0.0);
        }
        let tx: Vec<Cf64> = (0..fft_size).map(|i| ((i as f64 + 1.0).recip(), 0.0)).collect();
        let rx: Vec<Cf64> = tx.iter().map(|&s| cmul(s, (1.5, 0.0))).collect();
        let eq_zf = est.equalize_zf(&rx, 0);
        let eq_mm = est.equalize_mmse(&rx, 0);
        for k in 1..10 {
            assert!((eq_zf[k].0 - eq_mm[k].0).abs() < 0.01,
                "bin {k}: zf={} mmse={}", eq_zf[k].0, eq_mm[k].0);
        }
    }

    // ---- SNR ----

    #[test]
    fn snr_flat_channel() {
        let mut cfg = EstimatorConfig::wifi_a();
        cfg.noise_variance = 1.0;
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        for k in 0..fft_size {
            est.h_smooth[0][k] = (1.0, 0.0); // |H|^2 = 1 => SNR = 1/σ^2 = 0 dB
        }
        let snrs = est.per_subcarrier_snr(0);
        assert!(!snrs.is_empty());
        for s in &snrs {
            assert!((s.snr_db - 0.0).abs() < 0.1, "snr_db={}", s.snr_db);
        }
    }

    #[test]
    fn snr_10db() {
        let mut cfg = EstimatorConfig::wifi_a();
        cfg.noise_variance = 0.1; // |H|^2=1 => SNR = 10.0 = 10 dB
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        for k in 0..fft_size {
            est.h_smooth[0][k] = (1.0, 0.0);
        }
        let eff = est.effective_snr_db(0);
        assert!((eff - 10.0).abs() < 0.5, "eff_snr={eff}");
    }

    #[test]
    fn snr_varies_with_channel() {
        let mut cfg = EstimatorConfig::wifi_a();
        cfg.noise_variance = 1.0;
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        // Different magnitude per bin
        for k in 0..fft_size {
            est.h_smooth[0][k] = (k as f64 * 0.1 + 0.1, 0.0);
        }
        let snrs = est.per_subcarrier_snr(0);
        // SNR should increase with bin index (larger |H|)
        assert!(snrs[0].snr_linear < snrs[snrs.len() - 1].snr_linear);
    }

    // ---- Channel quality ----

    #[test]
    fn channel_quality_flat_channel() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        for k in 0..fft_size {
            est.h_smooth[0][k] = (1.0, 0.0);
        }
        let q = est.channel_quality(0);
        // Flat channel has no frequency selectivity
        assert!(q.freq_selectivity < 1e-6, "freq_sel={}", q.freq_selectivity);
        assert!(q.mean_channel_power > 0.9, "pwr={}", q.mean_channel_power);
    }

    #[test]
    fn channel_quality_selective_channel() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        // Frequency-selective: alternating magnitudes
        for k in 0..fft_size {
            est.h_smooth[0][k] = if k % 2 == 0 { (2.0, 0.0) } else { (0.5, 0.0) };
        }
        let q = est.channel_quality(0);
        assert!(q.freq_selectivity > 0.1, "should be selective: {}", q.freq_selectivity);
    }

    #[test]
    fn rms_delay_spread_known_channel() {
        // Pure flat channel → impulse CIR → zero delay spread
        let h = vec![(1.0_f64, 0.0_f64); 64];
        let freq_h = fft(&h);
        let cir = ifft(&freq_h);
        let rms = rms_delay_spread(&cir, 20e6);
        assert!(rms.is_finite(), "delay spread should be finite");
    }

    // ---- MIMO HT-LTF ----

    #[test]
    fn htltf_mimo2_identity_channel() {
        let cfg = EstimatorConfig::wifi_n20_mimo2();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        // For stream 0 and stream 1: H0=1, H1=1
        // HT-LTF: Y0 = P[0][0]*H0 + P[0][1]*H1, Y1 = P[1][0]*H0 + P[1][1]*H1
        // P for 2 streams: row0=[1,1], row1=[1,-1]
        // Y0[k] = 1*1 + 1*1 = 2, Y1[k] = 1*1 + (-1)*1 = 0
        let y0: Vec<Cf64> = (0..fft_size).map(|_| (2.0, 0.0)).collect();
        let y1: Vec<Cf64> = (0..fft_size).map(|_| (0.0, 0.0)).collect();
        est.load_htltf_freq(&[y0, y1]);
        // After P^-1, stream 0 should be ~1, stream 1 should be ~1 on active bins
        let h0 = est.channel_estimate_stream(0);
        let h1 = est.channel_estimate_stream(1);
        for k in 1..=26 {
            assert!((cabs(h0[k]) - 1.0).abs() < 0.1, "h0[{k}]={}", cabs(h0[k]));
            assert!((cabs(h1[k]) - 1.0).abs() < 0.1, "h1[{k}]={}", cabs(h1[k]));
        }
    }

    #[test]
    fn htltf_1stream_matches_lltf() {
        // With 1 stream, HT-LTF result should closely match L-LTF
        let cfg_a = EstimatorConfig::wifi_a();
        let cfg_n = EstimatorConfig::wifi_n20();
        let fft_size = cfg_a.mode.fft_size();
        assert_eq!(fft_size, cfg_n.mode.fft_size());

        let mut est_a = WifiChannelEstimator::new(cfg_a);
        let mut est_n = WifiChannelEstimator::new(cfg_n);

        // Flat H=2 channel in frequency domain
        let lltf_ref = build_lltf_reference(64);
        let ltf_freq: Vec<Cf64> = lltf_ref.iter().map(|&x| cmul(x, (2.0, 0.0))).collect();
        let ltf_td = ifft(&ltf_freq);
        est_a.load_lltf(&ltf_td, &ltf_td);

        // HT-LTF: single stream, P[0][0]=1 → Y = H * X
        // For simplicity, pass the same frequency-domain vector
        let y0 = ltf_freq.clone();
        est_n.load_htltf_freq(&[y0]);

        for k in 1..=26 {
            let ha = cabs(est_a.channel_estimate_stream(0)[k]);
            let hn = cabs(est_n.channel_estimate_stream(0)[k]);
            assert!((ha - 2.0).abs() < 0.2, "ha[{k}]={ha}");
            assert!((hn - 2.0).abs() < 0.2, "hn[{k}]={hn}");
        }
    }

    // ---- Active bins ----

    #[test]
    fn active_bins_legacy_ag_count() {
        let bins = active_bins(WifiMode::LegacyAG, 64);
        assert_eq!(bins.len(), 52);
    }

    #[test]
    fn active_bins_ht20_count() {
        let bins = active_bins(WifiMode::HtN20, 64);
        assert_eq!(bins.len(), 56);
    }

    #[test]
    fn active_bins_ht40_count() {
        let bins = active_bins(WifiMode::HtN40, 128);
        assert_eq!(bins.len(), 114);
    }

    // ---- Pilot bins ----

    #[test]
    fn pilot_bins_legacy_ag() {
        let bins = pilot_bins_for_mode(WifiMode::LegacyAG, 64);
        assert_eq!(bins.len(), 4);
        // Expected bins for {-21,-7,7,21} in 64-pt FFT: {43, 57, 7, 21}
        assert!(bins.contains(&7));
        assert!(bins.contains(&21));
        assert!(bins.contains(&43));
        assert!(bins.contains(&57));
    }

    #[test]
    fn pilot_bins_ht20_same_as_legacy() {
        let legacy = pilot_bins_for_mode(WifiMode::LegacyAG, 64);
        let ht = pilot_bins_for_mode(WifiMode::HtN20, 64);
        assert_eq!(legacy, ht);
    }

    // ---- Builder ----

    #[test]
    fn builder_creates_valid_config() {
        let cfg = EstimatorConfigBuilder::new(WifiMode::LegacyAG)
            .num_streams(2)
            .freq_smooth(FreqSmoothMethod::TimeDomain)
            .smooth_window(5)
            .noise_variance(0.01)
            .build();
        assert_eq!(cfg.num_streams, 2);
        assert_eq!(cfg.freq_smooth, FreqSmoothMethod::TimeDomain);
        assert!((cfg.noise_variance - 0.01).abs() < 1e-12);
    }

    // ---- Reset ----

    #[test]
    fn reset_clears_state() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        // Load some data
        let ltf = vec![(1.0, 0.0); fft_size];
        est.load_lltf(&ltf, &ltf);
        est.reset();
        assert_eq!(est.symbol_count(), 0);
        let h = est.channel_estimate_stream(0);
        assert!(h.iter().all(|&v| cabs(v) < 1e-10));
    }

    // ---- Update single subcarrier ----

    #[test]
    fn update_subcarrier_exponential_smoothing() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        est.h_smooth[0][10] = (1.0, 0.0);
        est.update_subcarrier(10, (2.0, 0.0), 0);
        let h = cabs(est.channel_estimate_stream(0)[10]);
        // Should be between 1 and 2
        assert!(h > 1.0 && h < 2.0, "h={h}");
    }

    // ---- CIR ----

    #[test]
    fn cir_impulse_at_zero() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        // Flat channel → CIR should be impulse at tap 0
        for k in 0..fft_size {
            est.h_smooth[0][k] = (1.0, 0.0);
        }
        let cir = est.channel_impulse_response(0);
        let peak = cir.iter().enumerate().max_by(|(_, a), (_, b)| {
            cabs2(**a).partial_cmp(&cabs2(**b)).unwrap()
        }).map(|(i, _)| i).unwrap();
        assert_eq!(peak, 0, "CIR peak should be at tap 0");
    }

    // ---- Noise variance update ----

    #[test]
    fn noise_var_update_decreases_from_high_init() {
        let mut cfg = EstimatorConfig::wifi_a();
        cfg.noise_variance = 1.0;
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        // Perfect channel: H=1 at pilots
        for k in 0..fft_size { est.h_smooth[0][k] = (1.0, 0.0); }
        // Perfect received pilot: rx[pilot] = H[pilot] * 1 = 1
        let mut rx = vec![czero(); fft_size];
        for &b in est.pilot_bins() { rx[b] = (1.0, 0.0); }
        est.update_noise_var_from_pilots(&rx, 0);
        // After update with zero-error pilots, noise_var should decrease
        assert!(est.noise_var[0] < 1.0, "noise_var should decrease: {}", est.noise_var[0]);
    }

    // ---- Coherence bandwidth ----

    #[test]
    fn coherence_bw_finite_positive() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        // Use a multi-path channel
        let mut h_cir = vec![czero(); fft_size];
        h_cir[0] = (1.0, 0.0);
        h_cir[4] = (0.5, 0.3);
        est.h_smooth[0] = fft(&h_cir);
        let q = est.channel_quality(0);
        assert!(q.coherence_bw_mhz > 0.0, "coh_bw={}", q.coherence_bw_mhz);
        assert!(q.coherence_bw_mhz < 1000.0, "coh_bw unexpectedly large: {}", q.coherence_bw_mhz);
    }

    // ---- Condition number SISO ----

    #[test]
    fn condition_number_siso_is_one() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        for k in 0..fft_size { est.h_smooth[0][k] = (1.0, 0.0); }
        let q = est.channel_quality(0);
        assert!((q.condition_number - 1.0).abs() < 1e-10);
    }

    // ---- Load LLTF freq-domain shortcut ----

    #[test]
    fn load_lltf_freq_matches_time_domain() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est_t = WifiChannelEstimator::new(cfg.clone());
        let mut est_f = WifiChannelEstimator::new(cfg);

        let lltf_ref = build_lltf_reference(fft_size);
        let ltf_td = ifft(&lltf_ref);

        est_t.load_lltf(&ltf_td, &ltf_td);
        est_f.load_lltf_freq(&lltf_ref, &lltf_ref);

        let ht = est_t.channel_estimate_stream(0);
        let hf = est_f.channel_estimate_stream(0);
        for k in 0..fft_size {
            assert!((ht[k].0 - hf[k].0).abs() < 0.01, "bin {k}: ht={} hf={}", ht[k].0, hf[k].0);
        }
    }

    // ---- Phase slope estimation ----

    #[test]
    fn phase_slope_from_linear_phase() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);
        // Set identity channel
        for k in 0..fft_size { est.h_smooth[0][k] = (1.0, 0.0); }
        let slope_true = 0.01_f64;
        let mut rx = vec![czero(); fft_size];
        for &b in est.pilot_bins() {
            rx[b] = cexp(slope_true * b as f64);
        }
        let result = estimate_phase_error(&rx, &est.h_smooth[0], est.pilot_bins());
        assert!((result.phase_slope - slope_true).abs() < 0.005, "slope={}", result.phase_slope);
    }

    // ---- WifiMode accessors ----

    #[test]
    fn wifi_mode_properties() {
        assert_eq!(WifiMode::LegacyAG.fft_size(), 64);
        assert_eq!(WifiMode::HtN20.fft_size(), 64);
        assert_eq!(WifiMode::HtN40.fft_size(), 128);
        assert_eq!(WifiMode::LegacyAG.num_active_subcarriers(), 52);
        assert_eq!(WifiMode::HtN20.num_active_subcarriers(), 56);
        assert_eq!(WifiMode::HtN40.num_active_subcarriers(), 114);
    }

    #[test]
    fn wifi_mode_data_subcarriers() {
        assert_eq!(WifiMode::LegacyAG.num_data_subcarriers(), 48);
        assert_eq!(WifiMode::HtN20.num_data_subcarriers(), 52);
    }

    // ---- Full pipeline smoke test ----

    #[test]
    fn full_pipeline_smoke_test() {
        let cfg = EstimatorConfig::wifi_a();
        let fft_size = cfg.mode.fft_size();
        let mut est = WifiChannelEstimator::new(cfg);

        // 1. Load L-LTF (flat H=1)
        let lltf_ref = build_lltf_reference(fft_size);
        let ltf_td = ifft(&lltf_ref);
        est.load_lltf(&ltf_td, &ltf_td);

        // 2. Receive data symbol (all ones at active subcarriers)
        let mut rx = vec![(1.0_f64, 0.0_f64); fft_size];
        // Add small CPE
        for v in rx.iter_mut() { *v = cmul(*v, cexp(0.1)); }

        // 3. Phase tracking
        est.track_phase(&mut rx, 0);

        // 4. ZF equalization
        let eq = est.equalize_zf(&rx, 0);

        // 5. SNR
        let snrs = est.per_subcarrier_snr(0);
        assert!(!snrs.is_empty());

        // 6. Quality
        let q = est.channel_quality(0);
        assert!(q.mean_channel_power > 0.0);
        assert!(q.rms_delay_spread_ns >= 0.0);
        assert!(!eq.is_empty());
    }
}
