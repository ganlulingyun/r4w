//! Iterative joint equalization and decoding for ISI channels.
//!
//! This module implements a turbo equalizer that iteratively exchanges soft
//! information (log-likelihood ratios) between a soft-input/soft-output (SISO)
//! equalizer and a SISO decoder. The equalizer removes inter-symbol interference
//! (ISI) introduced by multipath channels, while the decoder exploits code
//! structure to improve reliability. Each iteration refines the estimates,
//! converging toward the transmitted bit sequence.
//!
//! # Example
//!
//! ```
//! use r4w_core::turbo_equalizer::{TurboEqualizer, TurboEqualizerConfig, EqualizerType, DecoderType};
//!
//! // Define a simple 2-tap ISI channel: h = [1.0, 0.5]
//! let config = TurboEqualizerConfig {
//!     num_iterations: 3,
//!     equalizer_type: EqualizerType::Mmse,
//!     decoder_type: DecoderType::Identity,
//!     channel_taps: vec![(1.0, 0.0), (0.5, 0.0)],
//!     noise_variance: 0.1,
//! };
//!
//! let mut teq = TurboEqualizer::new(config);
//!
//! // Transmit BPSK symbols through the channel
//! let tx_bits = vec![true, false, true, true, false, false];
//! // Map bits to BPSK: true -> +1, false -> -1
//! let tx_symbols: Vec<(f64, f64)> = tx_bits.iter()
//!     .map(|&b| if b { (1.0, 0.0) } else { (-1.0, 0.0) })
//!     .collect();
//!
//! // Convolve with channel (simple ISI)
//! let mut rx: Vec<(f64, f64)> = vec![(0.0, 0.0); tx_symbols.len() + 1];
//! for (i, &(sr, si)) in tx_symbols.iter().enumerate() {
//!     rx[i].0 += sr * 1.0;
//!     rx[i].1 += si * 1.0;
//!     rx[i + 1].0 += sr * 0.5;
//!     rx[i + 1].1 += si * 0.5;
//! }
//! let rx = &rx[..tx_symbols.len()]; // truncate to original length
//!
//! let decoded = teq.equalize_decode(rx);
//! assert_eq!(decoded.len(), tx_bits.len());
//! // With a mild channel and 3 iterations, the result should match
//! assert_eq!(decoded, tx_bits);
//! ```

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Equalizer algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EqualizerType {
    /// Minimum Mean Square Error equalizer.
    Mmse,
    /// Maximum a-posteriori equalizer (simplified MAP).
    Map,
    /// Linear MMSE (frequency-domain approximation).
    LinearMmse,
}

/// Soft decoder algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DecoderType {
    /// Rate-1/3 repetition code: each bit transmitted three times.
    Repetition,
    /// Simple rate-1/2 convolutional code (constraint length 3).
    Convolutional,
    /// Identity (no coding) — passes LLRs through unchanged.
    Identity,
}

/// Configuration for the turbo equalizer.
#[derive(Debug, Clone)]
pub struct TurboEqualizerConfig {
    /// Number of turbo iterations.
    pub num_iterations: usize,
    /// Equalizer algorithm.
    pub equalizer_type: EqualizerType,
    /// Decoder algorithm.
    pub decoder_type: DecoderType,
    /// Channel impulse response (complex FIR taps).
    pub channel_taps: Vec<(f64, f64)>,
    /// Noise variance (sigma squared) of AWGN.
    pub noise_variance: f64,
}

/// Metrics collected after each turbo iteration.
#[derive(Debug, Clone)]
pub struct IterationMetrics {
    /// Estimated BER based on hard decisions of current LLRs.
    pub ber_estimate: f64,
    /// Mean squared error between equalized symbols and nearest BPSK point.
    pub mse: f64,
    /// Average magnitude of extrinsic information exchanged.
    pub extrinsic_info_magnitude: f64,
}

/// Iterative joint equalizer-decoder (turbo equalizer).
#[derive(Debug, Clone)]
pub struct TurboEqualizer {
    config: TurboEqualizerConfig,
    history: Vec<IterationMetrics>,
}

// ---------------------------------------------------------------------------
// Complex arithmetic helpers
// ---------------------------------------------------------------------------

#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

#[inline]
fn c_mag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

// ---------------------------------------------------------------------------
// Channel convolution helper
// ---------------------------------------------------------------------------

/// Convolve symbols with channel taps (FIR).
fn convolve(symbols: &[(f64, f64)], taps: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = symbols.len();
    let l = taps.len();
    let out_len = n + l - 1;
    let mut out = vec![(0.0, 0.0); out_len];
    for (i, &s) in symbols.iter().enumerate() {
        for (k, &t) in taps.iter().enumerate() {
            out[i + k] = c_add(out[i + k], c_mul(s, t));
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Soft decoder helpers
// ---------------------------------------------------------------------------

/// Repetition code (rate 1/3): encodes each bit as 3 copies.
fn repetition_encode(bits: &[bool]) -> Vec<bool> {
    let mut out = Vec::with_capacity(bits.len() * 3);
    for &b in bits {
        out.push(b);
        out.push(b);
        out.push(b);
    }
    out
}

/// Soft-decode repetition code: sum groups of 3 LLRs.
fn repetition_soft_decode(llr: &[f64]) -> Vec<f64> {
    let n = llr.len() / 3;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        out.push(llr[3 * i] + llr[3 * i + 1] + llr[3 * i + 2]);
    }
    out
}

/// Coded-domain extrinsic for repetition code.
/// For each coded position j in group k, extrinsic = sum of other positions' LLRs.
fn repetition_coded_extrinsic(channel_llr: &[f64]) -> Vec<f64> {
    let n = channel_llr.len() / 3;
    let mut out = vec![0.0; channel_llr.len()];
    for i in 0..n {
        let a = channel_llr[3 * i];
        let b = channel_llr[3 * i + 1];
        let c = channel_llr[3 * i + 2];
        out[3 * i] = b + c;
        out[3 * i + 1] = a + c;
        out[3 * i + 2] = a + b;
    }
    out
}

/// Simple rate-1/2 convolutional encoder (constraint length 3, generators [7, 5] octal).
fn convolutional_encode(bits: &[bool]) -> Vec<bool> {
    let mut out = Vec::with_capacity(bits.len() * 2);
    let mut sr: [bool; 2] = [false; 2];
    for &b in bits {
        let g1 = b ^ sr[0] ^ sr[1];
        let g2 = b ^ sr[1];
        out.push(g1);
        out.push(g2);
        sr[1] = sr[0];
        sr[0] = b;
    }
    out
}

/// Branch metric: positive LLR means bit=1 more likely.
#[inline]
fn branch_metric(g: bool, llr_obs: f64) -> f64 {
    if g { llr_obs / 2.0 } else { -llr_obs / 2.0 }
}

/// BCJR (max-log-MAP) decoder for rate-1/2 convolutional code.
/// Returns a-posteriori LLR for each info bit.
fn convolutional_soft_decode(llr: &[f64]) -> Vec<f64> {
    let n = llr.len() / 2;
    if n == 0 {
        return vec![];
    }

    let num_states = 4usize;
    let neg_inf = -1e30_f64;

    let trellis: [[(usize, bool, bool); 2]; 4] = {
        let mut t = [[(0usize, false, false); 2]; 4];
        for state in 0..4 {
            let sr0 = (state >> 1) & 1 != 0;
            let sr1 = state & 1 != 0;
            for input in 0..2 {
                let b = input != 0;
                let g1 = b ^ sr0 ^ sr1;
                let g2 = b ^ sr1;
                let next = ((b as usize) << 1) | (sr0 as usize);
                t[state][input] = (next, g1, g2);
            }
        }
        t
    };

    let mut alpha = vec![vec![neg_inf; num_states]; n + 1];
    alpha[0][0] = 0.0;

    for k in 0..n {
        let l0 = llr[2 * k];
        let l1 = llr[2 * k + 1];
        for s in 0..num_states {
            if alpha[k][s] <= neg_inf + 1.0 {
                continue;
            }
            for input in 0..2 {
                let (ns, g1, g2) = trellis[s][input];
                let bm = branch_metric(g1, l0) + branch_metric(g2, l1);
                let metric = alpha[k][s] + bm;
                alpha[k + 1][ns] = max_star(alpha[k + 1][ns], metric);
            }
        }
    }

    let mut beta = vec![vec![neg_inf; num_states]; n + 1];
    for s in 0..num_states {
        beta[n][s] = 0.0;
    }

    for k in (0..n).rev() {
        let l0 = llr[2 * k];
        let l1 = llr[2 * k + 1];
        for s in 0..num_states {
            for input in 0..2 {
                let (ns, g1, g2) = trellis[s][input];
                if beta[k + 1][ns] <= neg_inf + 1.0 {
                    continue;
                }
                let bm = branch_metric(g1, l0) + branch_metric(g2, l1);
                let metric = beta[k + 1][ns] + bm;
                beta[k][s] = max_star(beta[k][s], metric);
            }
        }
    }

    let mut app = Vec::with_capacity(n);
    for k in 0..n {
        let l0 = llr[2 * k];
        let l1 = llr[2 * k + 1];
        let mut llr_one = neg_inf;
        let mut llr_zero = neg_inf;
        for s in 0..num_states {
            if alpha[k][s] <= neg_inf + 1.0 {
                continue;
            }
            for input in 0..2 {
                let (ns, g1, g2) = trellis[s][input];
                if beta[k + 1][ns] <= neg_inf + 1.0 {
                    continue;
                }
                let bm = branch_metric(g1, l0) + branch_metric(g2, l1);
                let metric = alpha[k][s] + bm + beta[k + 1][ns];
                if input == 1 {
                    llr_one = max_star(llr_one, metric);
                } else {
                    llr_zero = max_star(llr_zero, metric);
                }
            }
        }
        app.push(llr_one - llr_zero);
    }

    app
}

/// Coded-domain extrinsic for convolutional code.
fn convolutional_coded_extrinsic(channel_llr: &[f64], decoder_app: &[f64]) -> Vec<f64> {
    let n = decoder_app.len();
    let coded_len = channel_llr.len();

    // Decoder extrinsic per info bit = APP - channel contribution
    let mut info_extrinsic = Vec::with_capacity(n);
    for k in 0..n {
        let channel_contribution = channel_llr[2 * k] + channel_llr[2 * k + 1];
        let ext = decoder_app[k] - channel_contribution;
        info_extrinsic.push(ext);
    }

    // Re-encode to coded domain
    let hard_bits: Vec<bool> = decoder_app.iter().map(|&l| l > 0.0).collect();
    let coded_bits = convolutional_encode(&hard_bits);
    let mut out = Vec::with_capacity(coded_len);
    for (i, &b) in coded_bits.iter().enumerate() {
        let info_idx = i / 2;
        let confidence = if info_idx < info_extrinsic.len() {
            info_extrinsic[info_idx].abs().min(20.0)
        } else {
            0.0
        };
        let sign = if b { 1.0 } else { -1.0 };
        out.push(sign * confidence * 0.5);
    }
    out.truncate(coded_len);
    out
}

/// Jacobian logarithm: max*(a,b) = max(a,b) + ln(1 + e^{-|a-b|}).
#[inline]
fn max_star(a: f64, b: f64) -> f64 {
    let max = if a > b { a } else { b };
    let diff = (a - b).abs();
    if diff > 30.0 {
        max
    } else {
        max + (1.0 + (-diff).exp()).ln()
    }
}

// ---------------------------------------------------------------------------
// TurboEqualizer implementation
// ---------------------------------------------------------------------------

impl TurboEqualizer {
    /// Create a new turbo equalizer from the given configuration.
    pub fn new(config: TurboEqualizerConfig) -> Self {
        Self {
            config,
            history: Vec::new(),
        }
    }

    /// Run iterative equalization and decoding on received samples.
    ///
    /// The turbo loop exchanges extrinsic information between the SISO
    /// equalizer and the SISO decoder. The equalizer's a-posteriori LLR
    /// equals the observation-based LLR plus the decoder prior. The
    /// extrinsic is the difference, which is then fed to the decoder.
    /// The decoder's extrinsic is computed and fed back as equalizer prior.
    pub fn equalize_decode(&mut self, rx_samples: &[(f64, f64)]) -> Vec<bool> {
        self.history.clear();

        let num_coded = rx_samples.len();
        if num_coded == 0 {
            return vec![];
        }

        let num_info = match self.config.decoder_type {
            DecoderType::Repetition => num_coded / 3,
            DecoderType::Convolutional => num_coded / 2,
            DecoderType::Identity => num_coded,
        };

        if num_info == 0 {
            return vec![];
        }

        // Decoder extrinsic fed to the equalizer as prior (starts at zero)
        let mut decoder_ext_coded = vec![0.0; num_coded];
        let mut decoded_bits = vec![false; num_info];

        for _iter in 0..self.config.num_iterations {
            // 1) Equalize: observation LLR (uses decoder extrinsic as prior
            //    for ISI cancellation and symbol mean estimation)
            let obs_llr = self.soft_equalize_observation(rx_samples, &decoder_ext_coded);

            // 2) Equalizer a-posteriori = observation LLR + decoder prior
            let eq_app: Vec<f64> = obs_llr
                .iter()
                .zip(decoder_ext_coded.iter())
                .map(|(o, p)| o + p)
                .collect();

            // 3) Equalizer extrinsic = a-posteriori - decoder prior = observation LLR
            //    (This is what we pass to the decoder as its channel input.)
            let eq_extrinsic = &obs_llr;

            // 4) Decode
            let dec_app = self.soft_decode(eq_extrinsic);

            // 5) Hard-decide info bits
            for (i, &l) in dec_app.iter().enumerate() {
                decoded_bits[i] = l > 0.0;
            }

            // 6) Decoder extrinsic in coded domain
            decoder_ext_coded =
                self.compute_decoder_extrinsic(eq_extrinsic, &dec_app, num_coded);

            // Metrics
            let ext_mag = obs_llr.iter().map(|x| x.abs()).sum::<f64>()
                / obs_llr.len().max(1) as f64;

            let mse = eq_app
                .iter()
                .map(|&l| {
                    let soft_sym = l.tanh();
                    let nearest = if soft_sym >= 0.0 { 1.0 } else { -1.0 };
                    (soft_sym - nearest) * (soft_sym - nearest)
                })
                .sum::<f64>()
                / eq_app.len().max(1) as f64;

            let ber = eq_app
                .iter()
                .filter(|&&l| l.abs() < 0.5)
                .count() as f64
                / eq_app.len().max(1) as f64;

            self.history.push(IterationMetrics {
                ber_estimate: ber,
                mse,
                extrinsic_info_magnitude: ext_mag,
            });
        }

        decoded_bits
    }

    /// Soft MMSE equalization producing LLR per coded bit.
    ///
    /// `rx` — received samples, `prior_llr` — decoder extrinsic (coded domain).
    /// Returns a-posteriori LLRs (observation + prior).
    pub fn soft_equalize(&mut self, rx: &[(f64, f64)], prior_llr: &[f64]) -> Vec<f64> {
        let obs = self.soft_equalize_observation(rx, prior_llr);
        // A-posteriori = observation + prior
        obs.iter()
            .zip(prior_llr.iter())
            .map(|(o, p)| (o + p).clamp(-50.0, 50.0))
            .collect()
    }

    /// Compute observation-only LLR (extrinsic from the equalizer's perspective).
    /// This is the LLR derived solely from the received signal, using the prior
    /// only for ISI cancellation, not added to the output.
    fn soft_equalize_observation(
        &self,
        rx: &[(f64, f64)],
        prior_llr: &[f64],
    ) -> Vec<f64> {
        let n = rx.len();
        if n == 0 {
            return vec![];
        }
        match self.config.equalizer_type {
            EqualizerType::Mmse | EqualizerType::Map => {
                self.mmse_soft_equalize(rx, prior_llr)
            }
            EqualizerType::LinearMmse => self.linear_mmse_equalize(rx, prior_llr),
        }
    }

    /// MMSE soft equalizer producing observation-only LLRs.
    fn mmse_soft_equalize(&self, rx: &[(f64, f64)], prior_llr: &[f64]) -> Vec<f64> {
        let n = rx.len();
        let h = &self.config.channel_taps;
        let sigma2 = self.config.noise_variance.max(1e-10);
        let l = h.len();

        let prior_means: Vec<(f64, f64)> = prior_llr
            .iter()
            .map(|&v| ((v / 2.0).tanh(), 0.0))
            .collect();

        let prior_vars: Vec<f64> = prior_llr
            .iter()
            .map(|&v| {
                let t = (v / 2.0).tanh();
                (1.0 - t * t).max(1e-10)
            })
            .collect();

        let h_energy: f64 = h.iter().map(|t| c_mag_sq(*t)).sum();
        let mut llr_out = vec![0.0; n];

        for k in 0..n {
            let mut mf = (0.0, 0.0);
            for (j, &tap) in h.iter().enumerate() {
                let rx_idx = k + j;
                if rx_idx < n {
                    mf = c_add(mf, c_mul(c_conj(tap), rx[rx_idx]));
                }
            }

            // ISI cancellation
            let mut interference = (0.0, 0.0);
            for (j, &tap) in h.iter().enumerate() {
                let obs_idx = k + j;
                for s in 0..n {
                    if s == k {
                        continue;
                    }
                    let ti = obs_idx as isize - s as isize;
                    if ti >= 0 && (ti as usize) < l {
                        let contrib = c_mul(h[ti as usize], prior_means[s]);
                        interference = c_add(interference, c_mul(c_conj(tap), contrib));
                    }
                }
            }

            let mf_clean = c_sub(mf, interference);

            let mut residual_var = 0.0;
            for (j, &tap) in h.iter().enumerate() {
                let obs_idx = k + j;
                for s in 0..n {
                    if s == k {
                        continue;
                    }
                    let ti = obs_idx as isize - s as isize;
                    if ti >= 0 && (ti as usize) < l {
                        residual_var +=
                            c_mag_sq(tap) * c_mag_sq(h[ti as usize]) * prior_vars[s];
                    }
                }
            }

            let denom = h_energy * h_energy + h_energy * (sigma2 + residual_var);
            let mmse_gain = if denom.abs() > 1e-30 {
                h_energy / denom
            } else {
                1.0
            };

            let x_hat = c_scale(mf_clean, mmse_gain);

            let post_var = if denom.abs() > 1e-30 {
                (sigma2 + residual_var) * h_energy / denom
            } else {
                1.0
            };

            let llr_val = if post_var.abs() > 1e-30 {
                2.0 * x_hat.0 / post_var
            } else {
                2.0 * x_hat.0 * 1e10
            };

            llr_out[k] = llr_val.clamp(-50.0, 50.0);
        }

        llr_out
    }

    /// Linear MMSE equalizer (matched-filter based).
    fn linear_mmse_equalize(&self, rx: &[(f64, f64)], _prior_llr: &[f64]) -> Vec<f64> {
        let n = rx.len();
        let h = &self.config.channel_taps;
        let sigma2 = self.config.noise_variance.max(1e-10);
        let h_energy: f64 = h.iter().map(|t| c_mag_sq(*t)).sum();

        let mut llr_out = vec![0.0; n];
        for k in 0..n {
            let mut mf = (0.0, 0.0);
            for (j, &tap) in h.iter().enumerate() {
                if k + j < n {
                    mf = c_add(mf, c_mul(c_conj(tap), rx[k + j]));
                }
            }
            let scale = 2.0 / (h_energy + sigma2).max(1e-30);
            llr_out[k] = (mf.0 * scale).clamp(-50.0, 50.0);
        }

        llr_out
    }

    /// Soft-input soft-output decoder.
    pub fn soft_decode(&self, channel_llr: &[f64]) -> Vec<f64> {
        match self.config.decoder_type {
            DecoderType::Repetition => repetition_soft_decode(channel_llr),
            DecoderType::Convolutional => convolutional_soft_decode(channel_llr),
            DecoderType::Identity => channel_llr.to_vec(),
        }
    }

    /// Return the iteration-by-iteration metrics history.
    pub fn iteration_history(&self) -> &[IterationMetrics] {
        &self.history
    }

    /// Update the channel taps.
    pub fn set_channel_taps(&mut self, taps: &[(f64, f64)]) {
        self.config.channel_taps = taps.to_vec();
    }

    /// Compute decoder extrinsic in coded domain.
    fn compute_decoder_extrinsic(
        &self,
        eq_extrinsic: &[f64],
        dec_app: &[f64],
        coded_len: usize,
    ) -> Vec<f64> {
        match self.config.decoder_type {
            DecoderType::Identity => vec![0.0; coded_len],
            DecoderType::Repetition => repetition_coded_extrinsic(eq_extrinsic),
            DecoderType::Convolutional => {
                convolutional_coded_extrinsic(eq_extrinsic, dec_app)
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn bpsk_map(bits: &[bool]) -> Vec<(f64, f64)> {
        bits.iter()
            .map(|&b| if b { (1.0, 0.0) } else { (-1.0, 0.0) })
            .collect()
    }

    fn apply_channel(symbols: &[(f64, f64)], taps: &[(f64, f64)]) -> Vec<(f64, f64)> {
        convolve(symbols, taps)
    }

    #[test]
    fn test_identity_no_isi() {
        let config = TurboEqualizerConfig {
            num_iterations: 1,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.01,
        };
        let mut teq = TurboEqualizer::new(config);
        let bits = vec![true, false, true, true, false];
        let symbols = bpsk_map(&bits);
        let decoded = teq.equalize_decode(&symbols);
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_identity_mild_isi() {
        let config = TurboEqualizerConfig {
            num_iterations: 5,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0), (0.3, 0.0)],
            noise_variance: 0.01,
        };
        let mut teq = TurboEqualizer::new(config);
        let bits = vec![true, false, true, true, false, true, false, false];
        let symbols = bpsk_map(&bits);
        let rx = apply_channel(&symbols, &[(1.0, 0.0), (0.3, 0.0)]);
        let rx_trunc = &rx[..bits.len()];
        let decoded = teq.equalize_decode(rx_trunc);
        assert_eq!(decoded.len(), bits.len());
        let errors: usize = decoded
            .iter()
            .zip(bits.iter())
            .filter(|(a, b)| a != b)
            .count();
        assert!(errors <= 2, "Too many errors: {}", errors);
    }

    #[test]
    fn test_identity_multiple_iterations_flat_channel() {
        let config = TurboEqualizerConfig {
            num_iterations: 5,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.01,
        };
        let mut teq = TurboEqualizer::new(config);
        let bits = vec![true, false, false, true, true, false];
        let symbols = bpsk_map(&bits);
        let decoded = teq.equalize_decode(&symbols);
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_repetition_decoder_no_isi() {
        let config = TurboEqualizerConfig {
            num_iterations: 2,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Repetition,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.01,
        };
        let mut teq = TurboEqualizer::new(config);
        let info_bits = vec![true, false, true];
        let coded_bits = repetition_encode(&info_bits);
        let symbols = bpsk_map(&coded_bits);
        let decoded = teq.equalize_decode(&symbols);
        assert_eq!(decoded, info_bits);
    }

    #[test]
    fn test_repetition_encode_decode_roundtrip() {
        let bits = vec![true, false, true, false, true];
        let coded = repetition_encode(&bits);
        assert_eq!(coded.len(), bits.len() * 3);
        let llr: Vec<f64> = coded
            .iter()
            .map(|&b| if b { 5.0 } else { -5.0 })
            .collect();
        let decoded_llr = repetition_soft_decode(&llr);
        let decoded: Vec<bool> = decoded_llr.iter().map(|&l| l > 0.0).collect();
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_convolutional_decoder_no_isi() {
        let config = TurboEqualizerConfig {
            num_iterations: 2,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Convolutional,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.01,
        };
        let mut teq = TurboEqualizer::new(config);
        let info_bits = vec![true, false, true, true, false];
        let coded_bits = convolutional_encode(&info_bits);
        let symbols = bpsk_map(&coded_bits);
        let decoded = teq.equalize_decode(&symbols);
        assert_eq!(decoded, info_bits);
    }

    #[test]
    fn test_convolutional_encode_decode_roundtrip() {
        let bits = vec![true, false, true, true, false, false, true];
        let coded = convolutional_encode(&bits);
        assert_eq!(coded.len(), bits.len() * 2);
        let llr: Vec<f64> = coded
            .iter()
            .map(|&b| if b { 10.0 } else { -10.0 })
            .collect();
        let decoded_llr = convolutional_soft_decode(&llr);
        let decoded: Vec<bool> = decoded_llr.iter().map(|&l| l > 0.0).collect();
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_iteration_history_populated() {
        let config = TurboEqualizerConfig {
            num_iterations: 4,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.1,
        };
        let mut teq = TurboEqualizer::new(config);
        let bits = vec![true, false, true];
        let symbols = bpsk_map(&bits);
        teq.equalize_decode(&symbols);
        assert_eq!(teq.iteration_history().len(), 4);
    }

    #[test]
    fn test_iteration_metrics_fields() {
        let config = TurboEqualizerConfig {
            num_iterations: 2,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0), (0.5, 0.0)],
            noise_variance: 0.1,
        };
        let mut teq = TurboEqualizer::new(config);
        let symbols = bpsk_map(&[true, false, true, false]);
        let rx = apply_channel(&symbols, &[(1.0, 0.0), (0.5, 0.0)]);
        teq.equalize_decode(&rx[..4]);
        for m in teq.iteration_history() {
            assert!(m.ber_estimate >= 0.0 && m.ber_estimate <= 1.0);
            assert!(m.mse >= 0.0);
            assert!(m.extrinsic_info_magnitude >= 0.0);
        }
    }

    #[test]
    fn test_convergence_metrics_are_finite() {
        let config = TurboEqualizerConfig {
            num_iterations: 6,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Repetition,
            channel_taps: vec![(1.0, 0.0), (0.4, 0.0)],
            noise_variance: 0.05,
        };
        let mut teq = TurboEqualizer::new(config);
        let info_bits = vec![true, false, true, true, false, true, false, true];
        let coded = repetition_encode(&info_bits);
        let symbols = bpsk_map(&coded);
        let rx = convolve(&symbols, &[(1.0, 0.0), (0.4, 0.0)]);
        teq.equalize_decode(&rx[..coded.len()]);
        let history = teq.iteration_history();
        assert_eq!(history.len(), 6);
        for m in history {
            assert!(m.mse.is_finite());
            assert!(m.ber_estimate.is_finite());
            assert!(m.extrinsic_info_magnitude.is_finite());
        }
    }

    #[test]
    fn test_set_channel_taps() {
        let config = TurboEqualizerConfig {
            num_iterations: 1,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.01,
        };
        let mut teq = TurboEqualizer::new(config);
        teq.set_channel_taps(&[(1.0, 0.0), (0.2, 0.1)]);
        let bits = vec![true, false];
        let symbols = bpsk_map(&bits);
        let decoded = teq.equalize_decode(&symbols);
        assert_eq!(decoded.len(), 2);
    }

    #[test]
    fn test_empty_input() {
        let config = TurboEqualizerConfig {
            num_iterations: 3,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.1,
        };
        let mut teq = TurboEqualizer::new(config);
        let decoded = teq.equalize_decode(&[]);
        assert!(decoded.is_empty());
        assert!(teq.iteration_history().is_empty());
    }

    #[test]
    fn test_soft_equalize_produces_correct_sign() {
        let config = TurboEqualizerConfig {
            num_iterations: 1,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.001,
        };
        let mut teq = TurboEqualizer::new(config);
        let bits = vec![true, false, true, false, true];
        let symbols = bpsk_map(&bits);
        let prior = vec![0.0; bits.len()];
        let llr = teq.soft_equalize(&symbols, &prior);
        for (i, &l) in llr.iter().enumerate() {
            if bits[i] {
                assert!(l > 0.0, "Bit {} should have positive LLR, got {}", i, l);
            } else {
                assert!(l < 0.0, "Bit {} should have negative LLR, got {}", i, l);
            }
        }
    }

    #[test]
    fn test_soft_decode_identity() {
        let config = TurboEqualizerConfig {
            num_iterations: 1,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.1,
        };
        let teq = TurboEqualizer::new(config);
        let input = vec![1.5, -2.0, 0.3];
        let output = teq.soft_decode(&input);
        assert_eq!(output, input);
    }

    #[test]
    fn test_soft_decode_repetition() {
        let config = TurboEqualizerConfig {
            num_iterations: 1,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Repetition,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.1,
        };
        let teq = TurboEqualizer::new(config);
        let input = vec![1.0, 0.8, 1.2, -0.5, -1.0, -0.7];
        let output = teq.soft_decode(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0] - 3.0).abs() < 1e-10);
        assert!((output[1] - (-2.2)).abs() < 1e-10);
    }

    #[test]
    fn test_linear_mmse_equalizer() {
        let config = TurboEqualizerConfig {
            num_iterations: 1,
            equalizer_type: EqualizerType::LinearMmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.01,
        };
        let mut teq = TurboEqualizer::new(config);
        let bits = vec![true, false, true, true, false];
        let symbols = bpsk_map(&bits);
        let decoded = teq.equalize_decode(&symbols);
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_map_equalizer_type() {
        let config = TurboEqualizerConfig {
            num_iterations: 2,
            equalizer_type: EqualizerType::Map,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(1.0, 0.0)],
            noise_variance: 0.01,
        };
        let mut teq = TurboEqualizer::new(config);
        let bits = vec![true, false, false, true];
        let symbols = bpsk_map(&bits);
        let decoded = teq.equalize_decode(&symbols);
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_complex_channel_taps() {
        let config = TurboEqualizerConfig {
            num_iterations: 3,
            equalizer_type: EqualizerType::Mmse,
            decoder_type: DecoderType::Identity,
            channel_taps: vec![(0.9, 0.1), (0.2, -0.1)],
            noise_variance: 0.01,
        };
        let mut teq = TurboEqualizer::new(config);
        let bits = vec![true, false, true, false, true, true];
        let symbols = bpsk_map(&bits);
        let rx = convolve(&symbols, &[(0.9, 0.1), (0.2, -0.1)]);
        let rx_trunc = &rx[..bits.len()];
        let decoded = teq.equalize_decode(rx_trunc);
        assert_eq!(decoded.len(), bits.len());
        let errors: usize = decoded
            .iter()
            .zip(bits.iter())
            .filter(|(a, b)| a != b)
            .count();
        assert!(errors <= 1, "Too many errors with complex channel: {}", errors);
    }

    #[test]
    fn test_convolve_helper() {
        let h = vec![(1.0, 0.0), (0.5, 0.0)];
        let x = vec![(1.0, 0.0), (-1.0, 0.0)];
        let y = convolve(&x, &h);
        assert_eq!(y.len(), 3);
        assert!((y[0].0 - 1.0).abs() < 1e-10);
        assert!((y[1].0 - (-0.5)).abs() < 1e-10);
        assert!((y[2].0 - (-0.5)).abs() < 1e-10);
    }

    #[test]
    fn test_max_star_function() {
        let a = 2.0;
        let b = 1.0;
        let result = max_star(a, b);
        assert!(result >= a);
        assert!(result > a);
        let result2 = max_star(3.0, 3.0);
        assert!((result2 - (3.0 + 2.0_f64.ln())).abs() < 1e-10);
    }

    #[test]
    fn test_repetition_coded_extrinsic() {
        let llr = vec![2.0, 3.0, 1.0, -1.0, -2.0, -3.0];
        let ext = repetition_coded_extrinsic(&llr);
        assert_eq!(ext.len(), 6);
        assert!((ext[0] - 4.0).abs() < 1e-10);
        assert!((ext[1] - 3.0).abs() < 1e-10);
        assert!((ext[2] - 5.0).abs() < 1e-10);
        assert!((ext[3] - (-5.0)).abs() < 1e-10);
        assert!((ext[4] - (-4.0)).abs() < 1e-10);
        assert!((ext[5] - (-3.0)).abs() < 1e-10);
    }
}
