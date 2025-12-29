#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

/// Error codes returned by R4W functions
enum class R4wError {
  /// Success (no error)
  Ok = 0,
  /// Null pointer passed
  NullPointer = 1,
  /// Invalid size or capacity
  InvalidSize = 2,
  /// Buffer is full
  BufferFull = 3,
  /// Buffer is empty
  BufferEmpty = 4,
  /// Invalid parameter value
  InvalidParameter = 5,
  /// Memory allocation failed
  AllocationFailed = 6,
  /// Operation not supported
  NotSupported = 7,
};

/// Waveform type enumeration
enum class R4wWaveformType {
  /// Binary Phase Shift Keying
  Bpsk = 0,
  /// Quadrature Phase Shift Keying
  Qpsk = 1,
  /// LoRa Chirp Spread Spectrum
  LoRa = 2,
};

/// Opaque chirp generator handle
struct R4wChirpGenerator;

/// Opaque FFT processor handle
struct R4wFftProcessor;

/// Opaque ring buffer handle
struct R4wRingBuffer;

/// Opaque waveform handle
struct R4wWaveform;

/// Complex I/Q sample (matches C's complex double or struct { double re, im; })
struct R4wComplex {
  /// Real (In-phase) component
  double re;
  /// Imaginary (Quadrature) component
  double im;
};

extern "C" {

/// Get R4W library version string
///
/// Returns a null-terminated string like "0.1.0"
const char *r4w_version();

/// Initialize the R4W library
///
/// Call once at program start. Currently a no-op but reserved for future use.
R4wError r4w_init();

/// Create a complex number from real and imaginary parts
R4wComplex r4w_complex_new(double re, double im);

/// Create a complex number from polar coordinates (magnitude, phase in radians)
R4wComplex r4w_complex_from_polar(double magnitude, double phase);

/// Get magnitude of a complex number
double r4w_complex_magnitude(R4wComplex c);

/// Get phase (argument) of a complex number in radians
double r4w_complex_phase(R4wComplex c);

/// Get power (magnitude squared) of a complex number
double r4w_complex_power(R4wComplex c);

/// Conjugate of a complex number
R4wComplex r4w_complex_conj(R4wComplex c);

/// Multiply two complex numbers
R4wComplex r4w_complex_mul(R4wComplex a, R4wComplex b);

/// Add two complex numbers
R4wComplex r4w_complex_add(R4wComplex a, R4wComplex b);

/// Compute average power of a buffer of samples
///
/// # Safety
/// `buffer` must point to at least `len` elements
double r4w_complex_average_power(const R4wComplex *buffer, uintptr_t len);

/// Normalize a buffer of samples to unit power (in-place)
///
/// # Safety
/// `buffer` must point to at least `len` elements
R4wError r4w_complex_normalize(R4wComplex *buffer, uintptr_t len);

/// Create a new FFT processor for the given size
///
/// Size should be a power of 2 for best performance.
/// Returns NULL on failure.
R4wFftProcessor *r4w_fft_new(uintptr_t size);

/// Free an FFT processor
///
/// # Safety
/// `fft` must be a valid pointer returned by `r4w_fft_new`
void r4w_fft_free(R4wFftProcessor *fft);

/// Get the size of an FFT processor
uintptr_t r4w_fft_size(const R4wFftProcessor *fft);

/// Compute forward FFT in-place
///
/// # Safety
/// - `fft` must be valid
/// - `buffer` must point to at least `len` elements
/// - `len` must match the FFT size
R4wError r4w_fft_forward(R4wFftProcessor *fft, R4wComplex *buffer, uintptr_t len);

/// Compute inverse FFT in-place (includes 1/N normalization)
///
/// # Safety
/// - `fft` must be valid
/// - `buffer` must point to at least `len` elements
/// - `len` must match the FFT size
R4wError r4w_fft_inverse(R4wFftProcessor *fft, R4wComplex *buffer, uintptr_t len);

/// Find the peak in an FFT magnitude spectrum
///
/// Returns the bin index, magnitude, and phase of the maximum.
///
/// # Safety
/// `buffer` must point to at least `len` elements
R4wError r4w_fft_find_peak(const R4wComplex *buffer,
                           uintptr_t len,
                           uintptr_t *out_bin,
                           double *out_magnitude,
                           double *out_phase);

/// Find peak with parabolic interpolation for sub-bin resolution
///
/// Returns interpolated bin index and magnitude.
///
/// # Safety
/// `buffer` must point to at least `len` elements
R4wError r4w_fft_find_peak_interpolated(const R4wComplex *buffer,
                                        uintptr_t len,
                                        double *out_bin,
                                        double *out_magnitude);

/// Compute magnitude spectrum
///
/// # Safety
/// - `input` must point to at least `len` elements
/// - `output` must point to at least `len` elements
R4wError r4w_fft_magnitude_spectrum(const R4wComplex *input, double *output, uintptr_t len);

/// Compute power spectrum in dB
///
/// # Safety
/// - `input` must point to at least `len` elements
/// - `output` must point to at least `len` elements
R4wError r4w_fft_power_spectrum_db(const R4wComplex *input, double *output, uintptr_t len);

/// Create a new chirp generator
///
/// # Arguments
/// * `spreading_factor` - LoRa spreading factor (5-12)
/// * `bandwidth` - Bandwidth in Hz (125000, 250000, or 500000)
/// * `oversample` - Oversampling factor (1-8, typically 1)
R4wChirpGenerator *r4w_chirp_new(uint8_t spreading_factor,
                                 uint32_t bandwidth,
                                 uintptr_t oversample);

/// Free a chirp generator
void r4w_chirp_free(R4wChirpGenerator *chirp);

/// Get samples per symbol for a chirp generator
uintptr_t r4w_chirp_samples_per_symbol(const R4wChirpGenerator *chirp);

/// Get bandwidth in Hz
double r4w_chirp_bandwidth(const R4wChirpGenerator *chirp);

/// Generate an upchirp and store in buffer
///
/// # Safety
/// `buffer` must point to at least `r4w_chirp_samples_per_symbol()` elements
R4wError r4w_chirp_generate_upchirp(const R4wChirpGenerator *chirp,
                                    R4wComplex *buffer,
                                    uintptr_t len);

/// Generate a downchirp and store in buffer
///
/// # Safety
/// `buffer` must point to at least `r4w_chirp_samples_per_symbol()` elements
R4wError r4w_chirp_generate_downchirp(const R4wChirpGenerator *chirp,
                                      R4wComplex *buffer,
                                      uintptr_t len);

/// Generate a modulated chirp for a given symbol
///
/// # Safety
/// `buffer` must point to at least `r4w_chirp_samples_per_symbol()` elements
R4wError r4w_chirp_modulate_symbol(const R4wChirpGenerator *chirp,
                                   uint16_t symbol,
                                   R4wComplex *buffer,
                                   uintptr_t len);

/// Create a new ring buffer
///
/// Capacity is rounded up to the next power of 2.
R4wRingBuffer *r4w_ringbuffer_new(uintptr_t capacity);

/// Free a ring buffer
void r4w_ringbuffer_free(R4wRingBuffer *rb);

/// Get ring buffer capacity
uintptr_t r4w_ringbuffer_capacity(const R4wRingBuffer *rb);

/// Get current number of elements in ring buffer
uintptr_t r4w_ringbuffer_len(const R4wRingBuffer *rb);

/// Check if ring buffer is empty
bool r4w_ringbuffer_is_empty(const R4wRingBuffer *rb);

/// Check if ring buffer is full
bool r4w_ringbuffer_is_full(const R4wRingBuffer *rb);

/// Push a single sample to the ring buffer
///
/// Returns R4wError::BufferFull if the buffer is full.
R4wError r4w_ringbuffer_push(const R4wRingBuffer *rb, R4wComplex sample);

/// Pop a single sample from the ring buffer
///
/// Returns R4wError::BufferEmpty if the buffer is empty.
R4wError r4w_ringbuffer_pop(const R4wRingBuffer *rb, R4wComplex *out);

/// Push multiple samples to the ring buffer
///
/// Returns the number of samples actually pushed.
///
/// # Safety
/// `buffer` must point to at least `len` elements
uintptr_t r4w_ringbuffer_push_slice(const R4wRingBuffer *rb,
                                    const R4wComplex *buffer,
                                    uintptr_t len);

/// Pop multiple samples from the ring buffer
///
/// Returns the number of samples actually popped.
///
/// # Safety
/// `buffer` must point to at least `len` elements
uintptr_t r4w_ringbuffer_pop_slice(const R4wRingBuffer *rb, R4wComplex *buffer, uintptr_t len);

/// Generate a complex sinusoid (cisoid) at the given frequency
///
/// # Arguments
/// * `frequency` - Frequency in Hz
/// * `sample_rate` - Sample rate in Hz
/// * `buffer` - Output buffer
/// * `len` - Number of samples to generate
///
/// # Safety
/// `buffer` must point to at least `len` elements
R4wError r4w_generate_tone(double frequency, double sample_rate, R4wComplex *buffer, uintptr_t len);

/// Add AWGN (Additive White Gaussian Noise) to a signal
///
/// # Arguments
/// * `buffer` - Signal buffer (modified in-place)
/// * `len` - Number of samples
/// * `snr_db` - Signal-to-noise ratio in dB
///
/// # Safety
/// `buffer` must point to at least `len` elements
R4wError r4w_add_awgn(R4wComplex *buffer, uintptr_t len, double snr_db);

/// Frequency shift a signal
///
/// # Arguments
/// * `buffer` - Signal buffer (modified in-place)
/// * `len` - Number of samples
/// * `frequency_offset` - Frequency shift in Hz
/// * `sample_rate` - Sample rate in Hz
///
/// # Safety
/// `buffer` must point to at least `len` elements
R4wError r4w_frequency_shift(R4wComplex *buffer,
                             uintptr_t len,
                             double frequency_offset,
                             double sample_rate);

/// Create a BPSK waveform
///
/// # Arguments
/// * `sample_rate` - Sample rate in Hz
/// * `symbol_rate` - Symbol rate in baud
R4wWaveform *r4w_waveform_bpsk_new(double sample_rate, double symbol_rate);

/// Create a QPSK waveform
///
/// # Arguments
/// * `sample_rate` - Sample rate in Hz
/// * `symbol_rate` - Symbol rate in baud
R4wWaveform *r4w_waveform_qpsk_new(double sample_rate, double symbol_rate);

/// Create a LoRa waveform
///
/// # Arguments
/// * `spreading_factor` - Spreading factor (5-12)
/// * `bandwidth` - Bandwidth in Hz (125000, 250000, 500000)
/// * `sample_rate` - Sample rate in Hz (typically 1MHz)
R4wWaveform *r4w_waveform_lora_new(uint8_t spreading_factor,
                                   uint32_t bandwidth,
                                   double sample_rate);

/// Free a waveform
void r4w_waveform_free(R4wWaveform *waveform);

/// Get waveform type
R4wWaveformType r4w_waveform_type(const R4wWaveform *waveform);

/// Get samples per symbol
uintptr_t r4w_waveform_samples_per_symbol(const R4wWaveform *waveform);

/// Get waveform name
///
/// Returns a pointer to a static string. Do not free.
const char *r4w_waveform_name(const R4wWaveform *waveform);

/// Modulate data into I/Q samples
///
/// # Arguments
/// * `waveform` - Waveform handle
/// * `data` - Input data bytes
/// * `data_len` - Number of data bytes
/// * `output` - Output buffer for I/Q samples
/// * `output_len` - Maximum output buffer size
/// * `samples_written` - Output: number of samples actually written
///
/// # Returns
/// * `R4wError::Ok` on success
/// * `R4wError::InvalidSize` if output buffer is too small
///
/// # Safety
/// * `data` must point to at least `data_len` bytes
/// * `output` must point to at least `output_len` elements
R4wError r4w_waveform_modulate(const R4wWaveform *waveform,
                               const uint8_t *data,
                               uintptr_t data_len,
                               R4wComplex *output,
                               uintptr_t output_len,
                               uintptr_t *samples_written);

/// Get required output buffer size for modulation
///
/// Returns the maximum number of samples that will be produced for the given input size.
uintptr_t r4w_waveform_modulate_size(const R4wWaveform *waveform, uintptr_t data_len);

/// Demodulate I/Q samples back to data
///
/// # Arguments
/// * `waveform` - Waveform handle
/// * `samples` - Input I/Q samples
/// * `samples_len` - Number of input samples
/// * `output` - Output buffer for demodulated data
/// * `output_len` - Maximum output buffer size
/// * `bytes_written` - Output: number of bytes actually written
///
/// # Safety
/// * `samples` must point to at least `samples_len` elements
/// * `output` must point to at least `output_len` bytes
R4wError r4w_waveform_demodulate(const R4wWaveform *waveform,
                                 const R4wComplex *samples,
                                 uintptr_t samples_len,
                                 uint8_t *output,
                                 uintptr_t output_len,
                                 uintptr_t *bytes_written);

}  // extern "C"
