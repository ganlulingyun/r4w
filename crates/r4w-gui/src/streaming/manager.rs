//! Stream manager - central coordinator for streaming playback

use super::{ChannelModel, PlaybackState, RecordingState, RingBuffer, StreamConfig, StreamSource, StreamStats, WaterfallState};
use super::{UdpSampleFormat, UdpStatus};
use r4w_core::types::IQSample;
use r4w_core::waveform::{Waveform, WaveformFactory};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use std::path::PathBuf;
use web_time::Instant;

#[cfg(not(target_arch = "wasm32"))]
use byteorder::{LittleEndian, WriteBytesExt};
#[cfg(not(target_arch = "wasm32"))]
use std::fs::File;
#[cfg(not(target_arch = "wasm32"))]
use std::io::BufWriter;

#[cfg(not(target_arch = "wasm32"))]
use super::udp::UdpReceiver;

/// Central manager for streaming state and playback
pub struct StreamManager {
    /// Current data source
    pub source: StreamSource,
    /// Playback state
    pub state: PlaybackState,
    /// Configuration
    pub config: StreamConfig,

    /// Ring buffer for time-domain display (RX samples in simulation mode)
    pub time_buffer: RingBuffer,
    /// Ring buffer for TX samples (simulation mode only)
    pub tx_buffer: RingBuffer,
    /// Waterfall state for spectrogram
    pub waterfall: WaterfallState,

    /// Last update timestamp
    last_update: Instant,

    /// File data cache (loaded samples)
    file_cache: Option<Vec<IQSample>>,

    /// Cached waveform instance for generator mode
    waveform_cache: Option<Box<dyn Waveform>>,
    /// Pre-generated sample buffer for streaming
    sample_buffer: Vec<IQSample>,
    /// Current position in sample buffer
    buffer_position: usize,

    /// Real-time statistics
    pub stats: StreamStats,

    /// Recording state
    pub recording_state: RecordingState,
    /// Buffer to collect samples during recording
    recording_buffer: Vec<IQSample>,
    /// Maximum recording samples (limit memory usage)
    max_recording_samples: usize,

    /// Simulation channel model
    pub sim_channel_model: ChannelModel,
    /// Simulation SNR in dB
    pub sim_snr_db: f64,
    /// Rician K-factor (ratio of LOS power to scattered power, in dB)
    pub sim_rician_k_db: f64,
    /// Carrier frequency offset in Hz
    pub sim_cfo_hz: f64,
    /// Phase accumulator for CFO (persists across apply_channel calls)
    sim_cfo_phase: f64,
    /// Phase noise standard deviation in degrees per sample (Wiener process increment)
    pub sim_phase_noise_deg: f64,
    /// Phase noise accumulator (random walk, persists across calls)
    sim_phase_noise_accum: f64,
    /// IQ amplitude imbalance in dB (Q gain relative to I)
    pub sim_iq_gain_db: f64,
    /// IQ phase imbalance in degrees (deviation from 90°)
    pub sim_iq_phase_deg: f64,
    /// DC offset on I channel (normalized to signal amplitude)
    pub sim_dc_offset_i: f64,
    /// DC offset on Q channel (normalized to signal amplitude)
    pub sim_dc_offset_q: f64,
    /// Demodulated bits from simulation (for display)
    pub sim_demod_bits: Vec<u8>,
    /// Original TX bits for comparison
    pub sim_tx_bits: Vec<u8>,
    /// Cumulative Bit Error Rate from simulation
    pub sim_ber: f64,
    /// Rolling window BER (last N bits)
    pub sim_ber_window: f64,
    /// Rolling window size for BER calculation
    sim_ber_window_size: usize,
    /// BER history for plotting (sampled periodically)
    pub sim_ber_history: Vec<f64>,
    /// Maximum BER history length
    sim_ber_history_max: usize,
    /// Total bit errors (cumulative)
    pub sim_bit_errors: usize,
    /// Bits since last BER history sample
    sim_bits_since_sample: usize,
    /// Sampling interval (bits per sample)
    sim_ber_sample_interval: usize,

    /// UDP receiver handle (native only)
    #[cfg(not(target_arch = "wasm32"))]
    udp_receiver: Option<UdpReceiver>,
}

impl std::fmt::Debug for StreamManager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StreamManager")
            .field("source", &self.source)
            .field("state", &self.state)
            .field("config", &self.config)
            .field("stats", &self.stats)
            .field("recording_state", &self.recording_state)
            .field("recording_samples", &self.recording_buffer.len())
            .field("sim_channel_model", &self.sim_channel_model)
            .field("sim_snr_db", &self.sim_snr_db)
            .field("sim_rician_k_db", &self.sim_rician_k_db)
            .field("sim_cfo_hz", &self.sim_cfo_hz)
            .field("sim_phase_noise_deg", &self.sim_phase_noise_deg)
            .field("sim_iq_gain_db", &self.sim_iq_gain_db)
            .field("sim_iq_phase_deg", &self.sim_iq_phase_deg)
            .field("sim_dc_offset_i", &self.sim_dc_offset_i)
            .field("sim_dc_offset_q", &self.sim_dc_offset_q)
            .field("sim_ber", &self.sim_ber)
            .field("sim_ber_window", &self.sim_ber_window)
            .field("sim_bit_errors", &self.sim_bit_errors)
            .field("sim_ber_history_len", &self.sim_ber_history.len())
            .finish()
    }
}

impl StreamManager {
    /// Maximum recording duration in samples (at 125 kHz = ~80 seconds = 10M samples)
    const DEFAULT_MAX_RECORDING_SAMPLES: usize = 10_000_000;

    /// Create a new stream manager with default configuration
    pub fn new(config: StreamConfig) -> Self {
        Self {
            source: StreamSource::None,
            state: PlaybackState::Stopped,
            time_buffer: RingBuffer::new(config.window_size),
            tx_buffer: RingBuffer::new(config.window_size),
            waterfall: WaterfallState::new(config.fft_size, config.waterfall_depth),
            config,
            last_update: Instant::now(),
            file_cache: None,
            waveform_cache: None,
            sample_buffer: Vec::new(),
            buffer_position: 0,
            stats: StreamStats::default(),
            recording_state: RecordingState::Idle,
            recording_buffer: Vec::new(),
            max_recording_samples: Self::DEFAULT_MAX_RECORDING_SAMPLES,
            sim_channel_model: ChannelModel::AWGN,
            sim_snr_db: 10.0,
            sim_rician_k_db: 6.0, // Default K=6dB (moderate LOS component)
            sim_cfo_hz: 0.0,      // No CFO by default
            sim_cfo_phase: 0.0,
            sim_phase_noise_deg: 0.0, // No phase noise by default
            sim_phase_noise_accum: 0.0,
            sim_iq_gain_db: 0.0,  // No IQ amplitude imbalance by default
            sim_iq_phase_deg: 0.0, // No IQ phase imbalance by default
            sim_dc_offset_i: 0.0, // No DC offset on I by default
            sim_dc_offset_q: 0.0, // No DC offset on Q by default
            sim_demod_bits: Vec::new(),
            sim_tx_bits: Vec::new(),
            sim_ber: 0.0,
            sim_ber_window: 0.0,
            sim_ber_window_size: 1000, // Last 1000 bits for rolling window
            sim_ber_history: Vec::with_capacity(200),
            sim_ber_history_max: 200, // Keep last 200 BER samples for plotting
            sim_bit_errors: 0,
            sim_bits_since_sample: 0,
            sim_ber_sample_interval: 256, // Sample BER every 256 bits
            #[cfg(not(target_arch = "wasm32"))]
            udp_receiver: None,
        }
    }

    /// Get list of available waveforms for generator
    pub fn available_waveforms() -> Vec<&'static str> {
        // Start with special streaming-only generators
        let mut waveforms = vec!["Tone", "Chirp", "Noise"];
        // Add all waveforms from factory
        waveforms.extend(WaveformFactory::list());
        waveforms
    }

    /// Load an IQ file for playback
    pub fn load_file(&mut self, path: PathBuf, samples: Vec<IQSample>) {
        let total = samples.len();
        self.file_cache = Some(samples);
        self.source = StreamSource::File {
            path,
            total_samples: total,
            position: 0,
            loop_enabled: true,
        };
        self.state = PlaybackState::Stopped;
        self.reset_buffers();
        self.stats.total_samples = total;
    }

    /// Start continuous waveform generator
    pub fn start_generator(&mut self, waveform: String) {
        // Try to create waveform from factory (for real waveforms)
        self.waveform_cache = WaveformFactory::create(&waveform, self.config.sample_rate);
        self.sample_buffer.clear();
        self.buffer_position = 0;

        // Pre-generate initial samples if using a real waveform
        if self.waveform_cache.is_some() {
            self.refill_sample_buffer();
        }

        self.source = StreamSource::Generator {
            waveform,
            phase: 0.0,
            samples_generated: 0,
        };
        self.state = PlaybackState::Stopped;
        self.reset_buffers();
        self.stats.total_samples = 0; // Infinite
    }

    /// Start TX→Channel→RX simulation mode
    pub fn start_simulation(&mut self, waveform: String, snr_db: f64) {
        // Create waveform from factory
        self.waveform_cache = WaveformFactory::create(&waveform, self.config.sample_rate);
        self.sample_buffer.clear();
        self.buffer_position = 0;
        self.sim_snr_db = snr_db;
        self.sim_tx_bits.clear();
        self.sim_demod_bits.clear();
        self.sim_ber = 0.0;
        self.sim_ber_window = 0.0;
        self.sim_ber_history.clear();
        self.sim_bit_errors = 0;
        self.sim_bits_since_sample = 0;

        // Pre-generate initial samples if using a real waveform
        if self.waveform_cache.is_some() {
            self.refill_simulation_buffer();
        }

        self.source = StreamSource::Simulation {
            waveform,
            snr_db,
            samples_generated: 0,
        };
        self.state = PlaybackState::Stopped;
        self.reset_buffers();
        self.stats.total_samples = 0; // Infinite
    }

    /// Check if currently in simulation mode
    pub fn is_simulation(&self) -> bool {
        matches!(self.source, StreamSource::Simulation { .. })
    }

    /// Check if currently receiving UDP
    pub fn is_udp(&self) -> bool {
        matches!(self.source, StreamSource::Udp { .. })
    }

    /// Start UDP listener on specified port (native only)
    #[cfg(not(target_arch = "wasm32"))]
    pub fn start_udp(&mut self, port: u16, format: UdpSampleFormat) -> Result<(), String> {
        // Stop any existing UDP receiver
        self.stop_udp();

        // Start new receiver
        let receiver = UdpReceiver::start(port, format)?;
        self.udp_receiver = Some(receiver);

        self.source = StreamSource::Udp {
            port,
            format,
            status: UdpStatus::Listening,
            packets_received: 0,
            samples_received: 0,
        };

        self.state = PlaybackState::Playing; // Auto-start for UDP
        self.reset_buffers();
        self.stats.total_samples = 0; // Infinite/unknown

        tracing::info!("Started UDP listener on port {} ({:?} format)", port, format);
        Ok(())
    }

    /// Stop UDP listener (native only)
    #[cfg(not(target_arch = "wasm32"))]
    pub fn stop_udp(&mut self) {
        if let Some(mut receiver) = self.udp_receiver.take() {
            receiver.stop();
            tracing::info!("Stopped UDP listener");
        }
        if matches!(self.source, StreamSource::Udp { .. }) {
            self.source = StreamSource::None;
            self.state = PlaybackState::Stopped;
        }
    }

    /// Get UDP statistics if active (native only)
    #[cfg(not(target_arch = "wasm32"))]
    pub fn udp_stats(&self) -> Option<&super::UdpStats> {
        self.udp_receiver.as_ref().map(|r| r.stats())
    }

    /// Refill the sample buffer with new waveform data
    fn refill_sample_buffer(&mut self) {
        if let Some(ref waveform) = self.waveform_cache {
            let mut rng = rand::thread_rng();

            // Generate 64 random bits for modulation
            let test_bits: Vec<u8> = (0..64).map(|_| rng.gen_range(0..=1)).collect();

            // Modulate to get samples
            let new_samples = waveform.modulate(&test_bits);
            self.sample_buffer.extend(new_samples);
        }
    }

    /// Refill simulation buffer with TX samples and generate RX samples through channel
    fn refill_simulation_buffer(&mut self) {
        // First, generate TX samples using the waveform (requires immutable borrow)
        let (tx_samples, tx_bits) = if let Some(ref waveform) = self.waveform_cache {
            let mut rng = rand::thread_rng();

            // Generate random bits for transmission
            let num_bits = 64;
            let tx_bits: Vec<u8> = (0..num_bits).map(|_| rng.gen_range(0..=1)).collect();

            // Modulate to get TX samples
            let tx_samples = waveform.modulate(&tx_bits);
            (tx_samples, tx_bits)
        } else {
            return;
        };

        // Store TX bits for BER calculation
        self.sim_tx_bits.extend(&tx_bits);

        // Apply channel model to get RX samples (requires mutable borrow for CFO phase)
        let rx_samples = self.apply_channel(&tx_samples);

        // Demodulate RX samples (requires immutable borrow again)
        if let Some(ref waveform) = self.waveform_cache {
            let demod_result = waveform.demodulate(&rx_samples);
            self.sim_demod_bits.extend(&demod_result.bits);
        }

        // Calculate BER statistics
        if !self.sim_tx_bits.is_empty() && !self.sim_demod_bits.is_empty() {
            let compare_len = self.sim_tx_bits.len().min(self.sim_demod_bits.len());

            // Count total errors for cumulative BER
            let errors: usize = self.sim_tx_bits.iter()
                .take(compare_len)
                .zip(self.sim_demod_bits.iter().take(compare_len))
                .filter(|(a, b)| a != b)
                .count();
            self.sim_bit_errors = errors;
            self.sim_ber = errors as f64 / compare_len as f64;

            // Calculate rolling window BER (last N bits)
            let window_start = compare_len.saturating_sub(self.sim_ber_window_size);
            let window_errors: usize = self.sim_tx_bits.iter()
                .skip(window_start)
                .take(compare_len - window_start)
                .zip(self.sim_demod_bits.iter().skip(window_start).take(compare_len - window_start))
                .filter(|(a, b)| a != b)
                .count();
            let window_len = compare_len - window_start;
            self.sim_ber_window = if window_len > 0 {
                window_errors as f64 / window_len as f64
            } else {
                0.0
            };

            // Update BER history (sample periodically)
            self.sim_bits_since_sample += tx_bits.len();
            if self.sim_bits_since_sample >= self.sim_ber_sample_interval {
                self.sim_bits_since_sample = 0;

                // Use window BER for history (more responsive than cumulative)
                self.sim_ber_history.push(self.sim_ber_window);

                // Trim history to max length
                if self.sim_ber_history.len() > self.sim_ber_history_max {
                    self.sim_ber_history.remove(0);
                }
            }
        }

        // Store both TX and RX samples - TX in sample_buffer, RX will be generated on demand
        // We store interleaved: TX sample followed by corresponding RX sample
        for (tx, rx) in tx_samples.iter().zip(rx_samples.iter()) {
            self.sample_buffer.push(*tx);
            self.sample_buffer.push(*rx);
        }
    }

    /// Apply channel model to samples (fading, noise, and CFO)
    fn apply_channel(&mut self, samples: &[IQSample]) -> Vec<IQSample> {
        // First apply fading and noise based on channel model
        let after_fading = match self.sim_channel_model {
            ChannelModel::Ideal => samples.to_vec(),
            ChannelModel::AWGN => {
                // Calculate noise power from SNR
                // SNR = signal_power / noise_power
                // noise_power = signal_power / 10^(SNR_dB/10)
                let signal_power: f64 = samples.iter()
                    .map(|s| s.norm_sqr())
                    .sum::<f64>() / samples.len().max(1) as f64;

                let snr_linear = 10.0_f64.powf(self.sim_snr_db / 10.0);
                let noise_power = signal_power / snr_linear;
                let noise_std = (noise_power / 2.0).sqrt(); // Divide by 2 for I and Q

                let normal = Normal::new(0.0, noise_std).unwrap_or(Normal::new(0.0, 0.001).unwrap());
                let mut rng = rand::thread_rng();

                samples.iter().map(|s| {
                    let noise_i = normal.sample(&mut rng);
                    let noise_q = normal.sample(&mut rng);
                    IQSample::new(s.re + noise_i, s.im + noise_q)
                }).collect()
            }
            ChannelModel::Rayleigh => {
                // Rayleigh fading: h = (X + jY) / sqrt(2) where X,Y ~ N(0,1)
                // This models multipath propagation with no line-of-sight component
                // Block fading: channel coefficient changes every coherence_samples
                let coherence_samples = 64; // Samples per fading block

                let signal_power: f64 = samples.iter()
                    .map(|s| s.norm_sqr())
                    .sum::<f64>() / samples.len().max(1) as f64;

                let snr_linear = 10.0_f64.powf(self.sim_snr_db / 10.0);
                let noise_power = signal_power / snr_linear;
                let noise_std = (noise_power / 2.0).sqrt();

                let channel_dist = Normal::new(0.0, 1.0 / std::f64::consts::SQRT_2).unwrap();
                let noise_dist = Normal::new(0.0, noise_std).unwrap_or(Normal::new(0.0, 0.001).unwrap());
                let mut rng = rand::thread_rng();

                let mut result = Vec::with_capacity(samples.len());
                let mut h_re = channel_dist.sample(&mut rng);
                let mut h_im = channel_dist.sample(&mut rng);

                for (i, s) in samples.iter().enumerate() {
                    // Update channel coefficient at coherence boundaries
                    if i > 0 && i % coherence_samples == 0 {
                        h_re = channel_dist.sample(&mut rng);
                        h_im = channel_dist.sample(&mut rng);
                    }

                    // Apply fading: y = h * x (complex multiplication)
                    // (h_re + j*h_im) * (s.re + j*s.im)
                    let faded_re = h_re * s.re - h_im * s.im;
                    let faded_im = h_re * s.im + h_im * s.re;

                    // Add AWGN
                    let noise_i = noise_dist.sample(&mut rng);
                    let noise_q = noise_dist.sample(&mut rng);

                    result.push(IQSample::new(faded_re + noise_i, faded_im + noise_q));
                }

                result
            }
            ChannelModel::Rician => {
                // Rician fading: h = sqrt(K/(K+1)) * e^(jθ) + sqrt(1/(K+1)) * (X + jY)
                // K is the ratio of LOS power to scattered power
                // When K=0, this becomes Rayleigh; as K→∞, approaches ideal
                let coherence_samples = 64;

                // Convert K from dB to linear
                let k_linear = 10.0_f64.powf(self.sim_rician_k_db / 10.0);

                // LOS and scatter components
                let los_amplitude = (k_linear / (k_linear + 1.0)).sqrt();
                let scatter_amplitude = (1.0 / (k_linear + 1.0)).sqrt();

                let signal_power: f64 = samples.iter()
                    .map(|s| s.norm_sqr())
                    .sum::<f64>() / samples.len().max(1) as f64;

                let snr_linear = 10.0_f64.powf(self.sim_snr_db / 10.0);
                let noise_power = signal_power / snr_linear;
                let noise_std = (noise_power / 2.0).sqrt();

                // Scatter component distribution (unit variance complex Gaussian)
                let scatter_dist = Normal::new(0.0, 1.0 / std::f64::consts::SQRT_2).unwrap();
                let noise_dist = Normal::new(0.0, noise_std).unwrap_or(Normal::new(0.0, 0.001).unwrap());
                let mut rng = rand::thread_rng();

                let mut result = Vec::with_capacity(samples.len());

                // Initialize channel: h = LOS + scatter
                // LOS component has random initial phase
                let mut los_phase: f64 = rng.gen::<f64>() * 2.0 * std::f64::consts::PI;
                let mut scatter_re = scatter_dist.sample(&mut rng);
                let mut scatter_im = scatter_dist.sample(&mut rng);

                for (i, s) in samples.iter().enumerate() {
                    // Update channel at coherence boundaries
                    if i > 0 && i % coherence_samples == 0 {
                        // LOS phase drifts slowly
                        los_phase += rng.gen::<f64>() * 0.1 - 0.05;
                        // Scatter component changes
                        scatter_re = scatter_dist.sample(&mut rng);
                        scatter_im = scatter_dist.sample(&mut rng);
                    }

                    // Combine LOS and scatter components
                    let h_re = los_amplitude * los_phase.cos() + scatter_amplitude * scatter_re;
                    let h_im = los_amplitude * los_phase.sin() + scatter_amplitude * scatter_im;

                    // Apply fading: y = h * x
                    let faded_re = h_re * s.re - h_im * s.im;
                    let faded_im = h_re * s.im + h_im * s.re;

                    // Add AWGN
                    let noise_i = noise_dist.sample(&mut rng);
                    let noise_q = noise_dist.sample(&mut rng);

                    result.push(IQSample::new(faded_re + noise_i, faded_im + noise_q));
                }

                result
            }
        };

        // Apply carrier frequency offset (CFO) if non-zero
        // CFO causes progressive phase rotation: y[n] = x[n] * e^(j * 2π * cfo * n * Ts)
        let after_cfo = if self.sim_cfo_hz.abs() < 1e-6 {
            // No CFO, pass through
            after_fading
        } else {
            let dt = 1.0 / self.config.sample_rate;
            let phase_increment = 2.0 * std::f64::consts::PI * self.sim_cfo_hz * dt;

            after_fading.iter().map(|s| {
                // Apply phase rotation
                let cos_phase = self.sim_cfo_phase.cos();
                let sin_phase = self.sim_cfo_phase.sin();

                // Complex multiplication: (s.re + j*s.im) * (cos + j*sin)
                let rotated_re = s.re * cos_phase - s.im * sin_phase;
                let rotated_im = s.re * sin_phase + s.im * cos_phase;

                // Update phase accumulator
                self.sim_cfo_phase += phase_increment;

                // Wrap phase to prevent numerical issues
                if self.sim_cfo_phase > 2.0 * std::f64::consts::PI {
                    self.sim_cfo_phase -= 2.0 * std::f64::consts::PI;
                } else if self.sim_cfo_phase < -2.0 * std::f64::consts::PI {
                    self.sim_cfo_phase += 2.0 * std::f64::consts::PI;
                }

                IQSample::new(rotated_re, rotated_im)
            }).collect()
        };

        // Apply phase noise if non-zero
        // Models oscillator instability as a Wiener process (random walk in phase)
        // θ[n] = θ[n-1] + Δθ where Δθ ~ N(0, σ²)
        let after_phase_noise = if self.sim_phase_noise_deg.abs() < 1e-6 {
            after_cfo
        } else {
            let mut rng = rand::thread_rng();
            let phase_std_rad = self.sim_phase_noise_deg * std::f64::consts::PI / 180.0;
            let noise_dist = rand_distr::Normal::new(0.0, phase_std_rad).unwrap();

            after_cfo.iter().map(|s| {
                // Add random phase increment (Wiener process)
                let delta_phase: f64 = noise_dist.sample(&mut rng);
                self.sim_phase_noise_accum += delta_phase;

                // Wrap phase to prevent numerical issues
                if self.sim_phase_noise_accum > 2.0 * std::f64::consts::PI {
                    self.sim_phase_noise_accum -= 2.0 * std::f64::consts::PI;
                } else if self.sim_phase_noise_accum < -2.0 * std::f64::consts::PI {
                    self.sim_phase_noise_accum += 2.0 * std::f64::consts::PI;
                }

                // Apply phase rotation
                let cos_phase = self.sim_phase_noise_accum.cos();
                let sin_phase = self.sim_phase_noise_accum.sin();
                let rotated_re = s.re * cos_phase - s.im * sin_phase;
                let rotated_im = s.re * sin_phase + s.im * cos_phase;

                IQSample::new(rotated_re, rotated_im)
            }).collect()
        };

        // Apply IQ imbalance if non-zero
        // Models gain and phase mismatch between I and Q branches in direct-conversion receivers
        // y_I = x_I (I branch is reference)
        // y_Q = g * (x_Q * cos(φ) + x_I * sin(φ))
        // where g = 10^(gain_db/20) and φ = phase_deg * π/180
        let has_iq_imbalance = self.sim_iq_gain_db.abs() > 0.001 || self.sim_iq_phase_deg.abs() > 0.01;

        let after_iq = if !has_iq_imbalance {
            after_phase_noise
        } else {
            // Convert parameters
            let g = 10.0_f64.powf(self.sim_iq_gain_db / 20.0); // Linear gain
            let phi = self.sim_iq_phase_deg * std::f64::consts::PI / 180.0; // Radians
            let cos_phi = phi.cos();
            let sin_phi = phi.sin();

            after_phase_noise.iter().map(|s| {
                // I branch unchanged (reference)
                let y_i = s.re;
                // Q branch has gain and phase imbalance
                let y_q = g * (s.im * cos_phi + s.re * sin_phi);
                IQSample::new(y_i, y_q)
            }).collect()
        };

        // Apply DC offset if non-zero
        // Models LO leakage and self-mixing in direct-conversion receivers
        // y_I = x_I + dc_i, y_Q = x_Q + dc_q
        let has_dc_offset = self.sim_dc_offset_i.abs() > 1e-6 || self.sim_dc_offset_q.abs() > 1e-6;

        if !has_dc_offset {
            after_iq
        } else {
            after_iq.iter().map(|s| {
                IQSample::new(s.re + self.sim_dc_offset_i, s.im + self.sim_dc_offset_q)
            }).collect()
        }
    }

    /// Start playback
    pub fn play(&mut self) {
        if matches!(self.source, StreamSource::None) {
            return;
        }
        self.state = PlaybackState::Playing;
        self.last_update = Instant::now();
    }

    /// Pause playback
    pub fn pause(&mut self) {
        self.state = PlaybackState::Paused;
    }

    /// Toggle play/pause
    pub fn toggle_play(&mut self) {
        match self.state {
            PlaybackState::Playing => self.pause(),
            PlaybackState::Paused | PlaybackState::Stopped => self.play(),
        }
    }

    /// Stop playback and reset position
    pub fn stop(&mut self) {
        self.state = PlaybackState::Stopped;
        self.reset_buffers();

        // Reset position for file playback
        if let StreamSource::File {
            ref mut position, ..
        } = self.source
        {
            *position = 0;
        }

        // Reset generator state
        if let StreamSource::Generator {
            ref mut phase,
            ref mut samples_generated,
            ..
        } = self.source
        {
            *phase = 0.0;
            *samples_generated = 0;
        }

        // Stop UDP receiver if active
        #[cfg(not(target_arch = "wasm32"))]
        if matches!(self.source, StreamSource::Udp { .. }) {
            self.stop_udp();
        }

        self.stats.current_position = 0;
    }

    /// Called each frame to advance playback.
    /// Returns true if display needs update.
    pub fn tick(&mut self) -> bool {
        if self.state != PlaybackState::Playing {
            return false;
        }

        let now = Instant::now();
        let elapsed = now.duration_since(self.last_update);
        self.last_update = now;

        // Calculate how many samples to consume based on playback speed
        let samples_per_second = self.config.sample_rate * self.config.playback_speed as f64;
        let samples_to_consume = (samples_per_second * elapsed.as_secs_f64()) as usize;

        if samples_to_consume == 0 {
            return false;
        }

        // Get samples from source
        let samples = self.get_samples(samples_to_consume);
        if samples.is_empty() {
            return false;
        }

        // Update buffers
        self.time_buffer.push_batch(&samples);
        self.waterfall.push_samples(&samples);
        self.update_stats(&samples);

        // Record samples if recording is active
        if self.recording_state == RecordingState::Recording {
            let remaining_capacity = self.max_recording_samples.saturating_sub(self.recording_buffer.len());
            if remaining_capacity > 0 {
                let samples_to_record = samples.len().min(remaining_capacity);
                self.recording_buffer.extend_from_slice(&samples[..samples_to_record]);
            }
            // Auto-stop if buffer is full
            if self.recording_buffer.len() >= self.max_recording_samples {
                self.recording_state = RecordingState::Idle;
            }
        }

        // Update effective sample rate
        self.stats.effective_sample_rate = samples_per_second;

        true
    }

    /// Get samples from the current source
    fn get_samples(&mut self, count: usize) -> Vec<IQSample> {
        match &mut self.source {
            StreamSource::None => Vec::new(),

            StreamSource::File {
                position,
                total_samples,
                loop_enabled,
                ..
            } => {
                if let Some(ref cache) = self.file_cache {
                    let mut result = Vec::with_capacity(count);
                    let mut remaining = count;

                    while remaining > 0 {
                        let available = (*total_samples).saturating_sub(*position);
                        let take = remaining.min(available);

                        if take > 0 {
                            result.extend_from_slice(&cache[*position..*position + take]);
                            *position += take;
                            remaining -= take;
                        }

                        if *position >= *total_samples {
                            if *loop_enabled {
                                *position = 0;
                            } else {
                                self.state = PlaybackState::Stopped;
                                break;
                            }
                        }
                    }

                    self.stats.current_position = *position;
                    result
                } else {
                    Vec::new()
                }
            }

            StreamSource::Generator { .. } => {
                // Check if we have a real waveform from the factory
                if self.waveform_cache.is_some() {
                    // Use the pre-generated sample buffer
                    let mut result = Vec::with_capacity(count);
                    let mut remaining = count;

                    while remaining > 0 {
                        // Refill buffer if needed
                        if self.buffer_position >= self.sample_buffer.len() {
                            self.refill_sample_buffer();
                            self.buffer_position = 0;
                        }

                        let available = self.sample_buffer.len() - self.buffer_position;
                        let take = remaining.min(available);

                        if take > 0 {
                            result.extend_from_slice(
                                &self.sample_buffer[self.buffer_position..self.buffer_position + take],
                            );
                            self.buffer_position += take;
                            remaining -= take;
                        }
                    }

                    // Update stats
                    if let StreamSource::Generator { samples_generated, .. } = &mut self.source {
                        *samples_generated += result.len() as u64;
                        self.stats.current_position = *samples_generated as usize;
                    }

                    result
                } else {
                    // Fallback to simple inline generators (Tone, Chirp, Noise)
                    let (waveform, mut phase) = if let StreamSource::Generator { waveform, phase, .. } = &self.source {
                        (waveform.clone(), *phase)
                    } else {
                        return Vec::new();
                    };

                    let samples = self.generate_simple_waveform(waveform, count, &mut phase);

                    // Update source with new phase
                    if let StreamSource::Generator { phase: ref mut p, samples_generated, .. } = &mut self.source {
                        *p = phase;
                        *samples_generated += samples.len() as u64;
                        self.stats.current_position = *samples_generated as usize;
                    }
                    samples
                }
            }

            StreamSource::Simulation { .. } => {
                // Use the pre-generated sample buffer (interleaved TX/RX)
                if self.waveform_cache.is_some() {
                    let mut tx_samples = Vec::with_capacity(count);
                    let mut rx_samples = Vec::with_capacity(count);
                    let mut remaining = count;

                    while remaining > 0 {
                        // Refill buffer if needed
                        if self.buffer_position >= self.sample_buffer.len() {
                            self.refill_simulation_buffer();
                            self.buffer_position = 0;
                        }

                        // Samples are stored interleaved: TX, RX, TX, RX, ...
                        let available_pairs = (self.sample_buffer.len() - self.buffer_position) / 2;
                        let take_pairs = remaining.min(available_pairs);

                        if take_pairs > 0 {
                            for i in 0..take_pairs {
                                let idx = self.buffer_position + i * 2;
                                tx_samples.push(self.sample_buffer[idx]);
                                rx_samples.push(self.sample_buffer[idx + 1]);
                            }
                            self.buffer_position += take_pairs * 2;
                            remaining -= take_pairs;
                        } else {
                            break;
                        }
                    }

                    // Update stats
                    if let StreamSource::Simulation { samples_generated, .. } = &mut self.source {
                        *samples_generated += rx_samples.len() as u64;
                        self.stats.current_position = *samples_generated as usize;
                    }

                    // Store TX samples for visualization (will be used by tick())
                    self.tx_buffer.push_batch(&tx_samples);

                    // Return RX samples (these go to time_buffer in tick())
                    rx_samples
                } else {
                    Vec::new()
                }
            }

            #[cfg(not(target_arch = "wasm32"))]
            StreamSource::Udp { packets_received, samples_received, status, .. } => {
                // Poll UDP receiver for samples
                if let Some(ref mut receiver) = self.udp_receiver {
                    if let Some(samples) = receiver.poll() {
                        // Update source statistics
                        let stats = receiver.stats();
                        *packets_received = stats.packets_received;
                        *samples_received = stats.samples_received;

                        // Update status based on receiver state
                        if receiver.is_running() {
                            *status = UdpStatus::Receiving;
                        }

                        // Check for errors
                        if receiver.last_error().is_some() {
                            *status = UdpStatus::Error;
                        }

                        self.stats.current_position = *samples_received as usize;
                        samples
                    } else {
                        // No samples available yet
                        Vec::new()
                    }
                } else {
                    Vec::new()
                }
            }

            // WASM fallback - UDP not available
            #[cfg(target_arch = "wasm32")]
            StreamSource::Udp { .. } => Vec::new(),
        }
    }

    /// Generate samples for simple waveforms (Tone, Chirp, Noise)
    /// Used as fallback when WaveformFactory doesn't have the waveform
    fn generate_simple_waveform(
        &self,
        waveform: String,
        count: usize,
        phase: &mut f64,
    ) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(count);
        let dt = 1.0 / self.config.sample_rate;

        match waveform.as_str() {
            "CW" | "Tone" => {
                // Simple 1 kHz tone
                let freq = 1000.0;
                for _ in 0..count {
                    let sample = IQSample::new(phase.cos(), phase.sin());
                    samples.push(sample);
                    *phase += 2.0 * std::f64::consts::PI * freq * dt;
                }
            }
            "Chirp" => {
                // Linear chirp from 0 to 10 kHz over 10ms
                let f0 = 0.0;
                let f1 = 10000.0;
                let chirp_duration = 0.01; // 10ms
                let chirp_rate = (f1 - f0) / chirp_duration;

                for _ in 0..count {
                    let t = (*phase / (2.0 * std::f64::consts::PI * f1)).fract() * chirp_duration;
                    let freq = f0 + chirp_rate * t;
                    let inst_phase = 2.0 * std::f64::consts::PI * (f0 * t + 0.5 * chirp_rate * t * t);
                    let sample = IQSample::new(inst_phase.cos(), inst_phase.sin());
                    samples.push(sample);
                    *phase += 2.0 * std::f64::consts::PI * freq * dt;
                }
            }
            "Noise" => {
                // White Gaussian noise
                use rand::Rng;
                let mut rng = rand::thread_rng();
                for _ in 0..count {
                    let i: f64 = rng.gen_range(-1.0..1.0);
                    let q: f64 = rng.gen_range(-1.0..1.0);
                    samples.push(IQSample::new(i, q));
                }
            }
            "BPSK" => {
                // Simple BPSK with random data
                use rand::Rng;
                let mut rng = rand::thread_rng();
                let symbol_rate = 1000.0;
                let sps = (self.config.sample_rate / symbol_rate) as usize;
                let carrier_freq = 5000.0;

                for i in 0..count {
                    // Change symbol every sps samples
                    let symbol = if (i / sps.max(1)) % 2 == 0 {
                        if rng.gen_bool(0.5) { 1.0 } else { -1.0 }
                    } else {
                        if rng.gen_bool(0.5) { 1.0 } else { -1.0 }
                    };
                    let sample = IQSample::new(
                        symbol * (phase.cos()),
                        symbol * (phase.sin()),
                    );
                    samples.push(sample);
                    *phase += 2.0 * std::f64::consts::PI * carrier_freq * dt;
                }
            }
            _ => {
                // Default: simple tone
                let freq = 1000.0;
                for _ in 0..count {
                    let sample = IQSample::new(phase.cos(), phase.sin());
                    samples.push(sample);
                    *phase += 2.0 * std::f64::consts::PI * freq * dt;
                }
            }
        }

        // Wrap phase to prevent overflow
        while *phase > 2.0 * std::f64::consts::PI * 1000.0 {
            *phase -= 2.0 * std::f64::consts::PI * 1000.0;
        }

        samples
    }

    /// Update real-time statistics from samples
    fn update_stats(&mut self, samples: &[IQSample]) {
        if samples.is_empty() {
            return;
        }

        // Average power
        let avg_power: f64 = samples.iter().map(|s| s.norm_sqr()).sum::<f64>() / samples.len() as f64;

        if avg_power > 0.0 {
            self.stats.average_power_db = (10.0 * avg_power.log10()) as f32;
        }

        // Peak power
        let peak_power = samples.iter().map(|s| s.norm_sqr()).fold(0.0_f64, f64::max);

        if peak_power > 0.0 {
            self.stats.peak_power_db = (10.0 * peak_power.log10()) as f32;
        }
    }

    /// Reset all buffers
    fn reset_buffers(&mut self) {
        self.time_buffer.clear();
        self.tx_buffer.clear();
        self.waterfall.clear();
        self.stats = StreamStats::default();
        self.sim_tx_bits.clear();
        self.sim_demod_bits.clear();
        self.sim_ber = 0.0;
        self.sim_ber_window = 0.0;
        self.sim_ber_history.clear();
        self.sim_bit_errors = 0;
        self.sim_bits_since_sample = 0;
        self.sim_cfo_phase = 0.0;
        self.sim_phase_noise_accum = 0.0;
    }

    /// Seek to position (file mode only)
    pub fn seek(&mut self, position: usize) {
        if let StreamSource::File {
            position: ref mut pos,
            total_samples,
            ..
        } = self.source
        {
            *pos = position.min(total_samples);
            self.stats.current_position = *pos;
            self.reset_buffers();
        }
    }

    /// Get playback progress as fraction 0.0 to 1.0
    pub fn progress(&self) -> f32 {
        match &self.source {
            StreamSource::File {
                position,
                total_samples,
                ..
            } => {
                if *total_samples > 0 {
                    *position as f32 / *total_samples as f32
                } else {
                    0.0
                }
            }
            _ => 0.0,
        }
    }

    /// Get elapsed time in seconds
    pub fn elapsed_seconds(&self) -> f64 {
        self.stats.current_position as f64 / self.config.sample_rate
    }

    /// Get total duration in seconds (file mode only)
    pub fn total_seconds(&self) -> f64 {
        self.stats.total_samples as f64 / self.config.sample_rate
    }

    /// Update configuration (may reset buffers)
    pub fn update_config(&mut self, config: StreamConfig) {
        let needs_resize = config.window_size != self.config.window_size
            || config.fft_size != self.config.fft_size
            || config.waterfall_depth != self.config.waterfall_depth;

        self.config = config;

        if needs_resize {
            self.time_buffer.resize(self.config.window_size);
            self.waterfall
                .resize(self.config.fft_size, self.config.waterfall_depth);
        }
    }

    /// Check if source is loaded
    pub fn has_source(&self) -> bool {
        !matches!(self.source, StreamSource::None)
    }

    /// Get source description for display
    pub fn source_description(&self) -> String {
        match &self.source {
            StreamSource::None => "No source".to_string(),
            StreamSource::File { path, .. } => {
                path.file_name()
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| "Unknown file".to_string())
            }
            StreamSource::Generator { waveform, .. } => format!("Generator: {}", waveform),
            StreamSource::Simulation { waveform, snr_db, .. } => format!("Sim: {} @ {:.0}dB", waveform, snr_db),
            StreamSource::Udp { port, format, .. } => format!("UDP:{} ({})", port, format.name()),
        }
    }

    // ============ Recording Methods ============

    /// Start recording samples
    pub fn start_recording(&mut self) {
        self.recording_buffer.clear();
        self.recording_state = RecordingState::Recording;
    }

    /// Stop recording
    pub fn stop_recording(&mut self) {
        self.recording_state = RecordingState::Idle;
    }

    /// Check if currently recording
    pub fn is_recording(&self) -> bool {
        self.recording_state == RecordingState::Recording
    }

    /// Get number of recorded samples
    pub fn recorded_samples(&self) -> usize {
        self.recording_buffer.len()
    }

    /// Get recording duration in seconds
    pub fn recording_duration(&self) -> f64 {
        self.recording_buffer.len() as f64 / self.config.sample_rate
    }

    /// Get recording buffer capacity remaining (as fraction 0.0 to 1.0)
    pub fn recording_capacity_remaining(&self) -> f32 {
        1.0 - (self.recording_buffer.len() as f32 / self.max_recording_samples as f32)
    }

    /// Clear the recording buffer
    pub fn clear_recording(&mut self) {
        self.recording_buffer.clear();
    }

    /// Get recorded samples as raw bytes (f32 little-endian interleaved I/Q pairs)
    pub fn get_recording_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(self.recording_buffer.len() * 8);
        for sample in &self.recording_buffer {
            bytes.extend_from_slice(&(sample.re as f32).to_le_bytes());
            bytes.extend_from_slice(&(sample.im as f32).to_le_bytes());
        }
        bytes
    }

    /// Save recorded samples to an IQ file (f32 little-endian interleaved)
    /// Note: This is used for native builds. For cross-platform, use `get_recording_bytes()`
    /// with the platform abstraction layer.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn save_recording(&self, path: &PathBuf) -> Result<usize, String> {
        if self.recording_buffer.is_empty() {
            return Err("No samples recorded".to_string());
        }

        let file = File::create(path).map_err(|e| format!("Failed to create file: {}", e))?;
        let mut writer = BufWriter::new(file);

        for sample in &self.recording_buffer {
            writer
                .write_f32::<LittleEndian>(sample.re as f32)
                .map_err(|e| format!("Failed to write I component: {}", e))?;
            writer
                .write_f32::<LittleEndian>(sample.im as f32)
                .map_err(|e| format!("Failed to write Q component: {}", e))?;
        }

        Ok(self.recording_buffer.len())
    }
}

impl Default for StreamManager {
    fn default() -> Self {
        Self::new(StreamConfig::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use byteorder::{LittleEndian, ReadBytesExt};
    use std::io::BufReader;

    #[test]
    fn test_recording_roundtrip() {
        // Create manager and start a simple generator
        let mut manager = StreamManager::new(StreamConfig {
            sample_rate: 10000.0,
            ..Default::default()
        });

        // Start generator (Tone - simple waveform)
        manager.start_generator("Tone".to_string());
        manager.play();

        // Start recording
        manager.start_recording();
        assert!(manager.is_recording());
        assert_eq!(manager.recorded_samples(), 0);

        // Simulate some ticks to generate and record samples
        for _ in 0..10 {
            // Manually add some samples to simulate playback
            let samples: Vec<IQSample> = (0..100)
                .map(|i| {
                    let phase = (i as f64) * 0.1;
                    IQSample::new(phase.cos(), phase.sin())
                })
                .collect();

            // Push to recording buffer (simulating what tick() does)
            manager.recording_buffer.extend_from_slice(&samples);
        }

        // Stop recording
        manager.stop_recording();
        assert!(!manager.is_recording());
        assert_eq!(manager.recorded_samples(), 1000);

        // Save to temp file
        let temp_dir = std::env::temp_dir();
        let temp_file = temp_dir.join("test_recording.iq");

        let saved_count = manager.save_recording(&temp_file).expect("Failed to save");
        assert_eq!(saved_count, 1000);

        // Verify file exists and has correct size (1000 samples * 8 bytes)
        let metadata = std::fs::metadata(&temp_file).expect("File should exist");
        assert_eq!(metadata.len(), 1000 * 8);

        // Load file back and verify samples
        let file = std::fs::File::open(&temp_file).expect("Failed to open");
        let mut reader = BufReader::new(file);
        let mut loaded_samples = Vec::new();

        loop {
            let i = match reader.read_f32::<LittleEndian>() {
                Ok(v) => v as f64,
                Err(_) => break,
            };
            let q = match reader.read_f32::<LittleEndian>() {
                Ok(v) => v as f64,
                Err(_) => break,
            };
            loaded_samples.push(IQSample::new(i, q));
        }

        assert_eq!(loaded_samples.len(), 1000);

        // Verify first few samples match (within f32 precision)
        for i in 0..10 {
            let original = &manager.recording_buffer[i];
            let loaded = &loaded_samples[i];
            assert!((original.re - loaded.re).abs() < 1e-6, "I mismatch at {}", i);
            assert!((original.im - loaded.im).abs() < 1e-6, "Q mismatch at {}", i);
        }

        // Clean up
        std::fs::remove_file(&temp_file).ok();
    }

    #[test]
    fn test_recording_capacity_limit() {
        let mut manager = StreamManager::new(StreamConfig::default());

        // Set a small limit for testing
        manager.max_recording_samples = 100;

        manager.start_recording();

        // Add more samples than the limit
        let samples: Vec<IQSample> = (0..150)
            .map(|i| IQSample::new(i as f64, i as f64))
            .collect();

        // Simulate what tick() does - respects capacity
        let remaining = manager.max_recording_samples.saturating_sub(manager.recording_buffer.len());
        let to_record = samples.len().min(remaining);
        manager.recording_buffer.extend_from_slice(&samples[..to_record]);

        // Should be capped at max
        assert_eq!(manager.recorded_samples(), 100);
    }

    #[test]
    fn test_save_empty_recording() {
        let manager = StreamManager::new(StreamConfig::default());
        let temp_file = std::env::temp_dir().join("empty_recording.iq");

        let result = manager.save_recording(&temp_file);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), "No samples recorded");
    }

    #[test]
    fn test_clear_recording() {
        let mut manager = StreamManager::new(StreamConfig::default());

        // Add some samples
        manager.start_recording();
        manager.recording_buffer.extend_from_slice(&[
            IQSample::new(1.0, 2.0),
            IQSample::new(3.0, 4.0),
        ]);
        assert_eq!(manager.recorded_samples(), 2);

        // Clear
        manager.clear_recording();
        assert_eq!(manager.recorded_samples(), 0);
    }
}
