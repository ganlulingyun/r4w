/**
 * @file r4w.hpp
 * @brief C++ Wrapper for R4W DSP Library
 *
 * This header provides idiomatic C++ wrappers for the R4W C API.
 * Features:
 * - RAII resource management (automatic cleanup)
 * - Exception-based error handling
 * - STL-compatible interfaces
 * - Type-safe waveform abstraction
 *
 * @example
 * @code
 * #include <r4w.hpp>
 *
 * // Create a BPSK waveform and modulate data
 * r4w::Waveform waveform = r4w::Waveform::bpsk(1e6, 9600);
 * std::vector<uint8_t> data = {0x48, 0x65, 0x6C, 0x6C, 0x6F};
 * auto samples = waveform.modulate(data);
 *
 * // Process with FFT
 * r4w::FFT fft(1024);
 * fft.forward(samples);
 *
 * // Use ring buffer for streaming
 * r4w::RingBuffer<r4w::Complex> ring(4096);
 * ring.push_slice(samples);
 * @endcode
 *
 * Requires C++17 or later.
 * Link with: -lr4w
 */

#ifndef R4W_HPP
#define R4W_HPP

#include <cstdint>
#include <cstddef>
#include <complex>
#include <vector>
#include <string>
#include <stdexcept>
#include <memory>
#include <optional>

// Include the C header
extern "C" {
#include "r4w.h"
}

namespace r4w {

// ============================================================================
// Exception Types
// ============================================================================

/**
 * @brief Base exception for R4W errors
 */
class Error : public std::runtime_error {
public:
    explicit Error(const std::string& msg) : std::runtime_error(msg) {}
    explicit Error(r4w_error_t code)
        : std::runtime_error(error_message(code)), code_(code) {}

    r4w_error_t code() const noexcept { return code_; }

private:
    r4w_error_t code_ = R4W_ERROR_OK;

    static std::string error_message(r4w_error_t code) {
        switch (code) {
            case R4W_ERROR_OK: return "Success";
            case R4W_ERROR_NULL_POINTER: return "Null pointer";
            case R4W_ERROR_INVALID_SIZE: return "Invalid size";
            case R4W_ERROR_BUFFER_FULL: return "Buffer full";
            case R4W_ERROR_BUFFER_EMPTY: return "Buffer empty";
            case R4W_ERROR_INVALID_PARAMETER: return "Invalid parameter";
            default: return "Unknown error";
        }
    }
};

// ============================================================================
// Complex Type Alias
// ============================================================================

/**
 * @brief Complex sample type (matches r4w_complex_t)
 */
using Complex = std::complex<double>;

inline Complex to_complex(r4w_complex_t c) {
    return Complex(c.re, c.im);
}

inline r4w_complex_t from_complex(const Complex& c) {
    return r4w_complex_new(c.real(), c.imag());
}

// ============================================================================
// FFT Processor
// ============================================================================

/**
 * @brief FFT processor with RAII resource management
 *
 * Provides forward and inverse FFT with automatic cleanup.
 */
class FFT {
public:
    /**
     * @brief Create an FFT processor
     * @param size FFT size (should be power of 2)
     * @throws Error if allocation fails
     */
    explicit FFT(size_t size) : handle_(r4w_fft_new(size)) {
        if (!handle_) {
            throw Error("Failed to create FFT processor");
        }
    }

    ~FFT() {
        if (handle_) {
            r4w_fft_free(handle_);
        }
    }

    // Non-copyable, movable
    FFT(const FFT&) = delete;
    FFT& operator=(const FFT&) = delete;
    FFT(FFT&& other) noexcept : handle_(other.handle_) { other.handle_ = nullptr; }
    FFT& operator=(FFT&& other) noexcept {
        if (this != &other) {
            if (handle_) r4w_fft_free(handle_);
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    /**
     * @brief Compute forward FFT in-place
     * @param data Complex samples to transform
     */
    void forward(std::vector<Complex>& data) {
        auto err = r4w_fft_forward(handle_,
            reinterpret_cast<r4w_complex_t*>(data.data()), data.size());
        if (err != R4W_ERROR_OK) throw Error(err);
    }

    /**
     * @brief Compute inverse FFT in-place
     * @param data Complex samples to transform
     */
    void inverse(std::vector<Complex>& data) {
        auto err = r4w_fft_inverse(handle_,
            reinterpret_cast<r4w_complex_t*>(data.data()), data.size());
        if (err != R4W_ERROR_OK) throw Error(err);
    }

    /**
     * @brief Find peak in spectrum
     * @param data Complex spectrum data
     * @return Tuple of (bin index, magnitude, phase)
     */
    std::tuple<size_t, double, double> find_peak(const std::vector<Complex>& data) const {
        size_t bin;
        double mag, phase;
        auto err = r4w_fft_find_peak(
            reinterpret_cast<const r4w_complex_t*>(data.data()), data.size(),
            &bin, &mag, &phase);
        if (err != R4W_ERROR_OK) throw Error(err);
        return {bin, mag, phase};
    }

    /**
     * @brief Compute magnitude spectrum
     * @param data Complex spectrum data
     * @return Vector of magnitudes
     */
    std::vector<double> magnitude_spectrum(const std::vector<Complex>& data) const {
        std::vector<double> mag(data.size());
        auto err = r4w_fft_magnitude_spectrum(
            reinterpret_cast<const r4w_complex_t*>(data.data()),
            mag.data(), data.size());
        if (err != R4W_ERROR_OK) throw Error(err);
        return mag;
    }

    /**
     * @brief Compute power spectrum in dB
     * @param data Complex spectrum data
     * @return Vector of power values in dB
     */
    std::vector<double> power_spectrum_db(const std::vector<Complex>& data) const {
        std::vector<double> power(data.size());
        auto err = r4w_fft_power_spectrum_db(
            reinterpret_cast<const r4w_complex_t*>(data.data()),
            power.data(), data.size());
        if (err != R4W_ERROR_OK) throw Error(err);
        return power;
    }

private:
    r4w_fft_t* handle_;
};

// ============================================================================
// Chirp Generator
// ============================================================================

/**
 * @brief LoRa chirp signal generator
 *
 * Generates up/down chirps for LoRa modulation.
 */
class ChirpGenerator {
public:
    /**
     * @brief Create a chirp generator
     * @param spreading_factor LoRa spreading factor (5-12)
     * @param bandwidth Bandwidth in Hz (125000, 250000, 500000)
     * @param oversample Oversampling factor (typically 1)
     * @throws Error if parameters are invalid
     */
    ChirpGenerator(uint8_t spreading_factor, uint32_t bandwidth, size_t oversample = 1)
        : handle_(r4w_chirp_new(spreading_factor, bandwidth, oversample)) {
        if (!handle_) {
            throw Error("Failed to create chirp generator");
        }
    }

    ~ChirpGenerator() {
        if (handle_) {
            r4w_chirp_free(handle_);
        }
    }

    // Non-copyable, movable
    ChirpGenerator(const ChirpGenerator&) = delete;
    ChirpGenerator& operator=(const ChirpGenerator&) = delete;
    ChirpGenerator(ChirpGenerator&& other) noexcept : handle_(other.handle_) { other.handle_ = nullptr; }
    ChirpGenerator& operator=(ChirpGenerator&& other) noexcept {
        if (this != &other) {
            if (handle_) r4w_chirp_free(handle_);
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    /**
     * @brief Get samples per symbol
     */
    size_t samples_per_symbol() const {
        return r4w_chirp_samples_per_symbol(handle_);
    }

    /**
     * @brief Get bandwidth in Hz
     */
    double bandwidth() const {
        return r4w_chirp_bandwidth(handle_);
    }

    /**
     * @brief Generate an upchirp
     * @return Vector of complex samples
     */
    std::vector<Complex> generate_upchirp() const {
        size_t len = samples_per_symbol();
        std::vector<Complex> samples(len);
        auto err = r4w_chirp_generate_upchirp(handle_,
            reinterpret_cast<r4w_complex_t*>(samples.data()), len);
        if (err != R4W_ERROR_OK) throw Error(err);
        return samples;
    }

    /**
     * @brief Generate a downchirp
     * @return Vector of complex samples
     */
    std::vector<Complex> generate_downchirp() const {
        size_t len = samples_per_symbol();
        std::vector<Complex> samples(len);
        auto err = r4w_chirp_generate_downchirp(handle_,
            reinterpret_cast<r4w_complex_t*>(samples.data()), len);
        if (err != R4W_ERROR_OK) throw Error(err);
        return samples;
    }

    /**
     * @brief Modulate a symbol
     * @param symbol Symbol value (0 to 2^SF - 1)
     * @return Vector of complex samples
     */
    std::vector<Complex> modulate_symbol(uint16_t symbol) const {
        size_t len = samples_per_symbol();
        std::vector<Complex> samples(len);
        auto err = r4w_chirp_modulate_symbol(handle_,
            symbol, reinterpret_cast<r4w_complex_t*>(samples.data()), len);
        if (err != R4W_ERROR_OK) throw Error(err);
        return samples;
    }

private:
    r4w_chirp_t* handle_;
};

// ============================================================================
// Ring Buffer
// ============================================================================

/**
 * @brief Lock-free SPSC ring buffer for real-time streaming
 *
 * Thread-safe for single producer, single consumer pattern.
 * @tparam T Must be Complex (r4w_complex_t compatible)
 */
template<typename T = Complex>
class RingBuffer {
    static_assert(std::is_same_v<T, Complex>, "RingBuffer only supports Complex type");

public:
    /**
     * @brief Create a ring buffer
     * @param capacity Maximum number of elements
     * @throws Error if allocation fails
     */
    explicit RingBuffer(size_t capacity) : handle_(r4w_ringbuffer_new(capacity)) {
        if (!handle_) {
            throw Error("Failed to create ring buffer");
        }
    }

    ~RingBuffer() {
        if (handle_) {
            r4w_ringbuffer_free(handle_);
        }
    }

    // Non-copyable, movable
    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;
    RingBuffer(RingBuffer&& other) noexcept : handle_(other.handle_) { other.handle_ = nullptr; }
    RingBuffer& operator=(RingBuffer&& other) noexcept {
        if (this != &other) {
            if (handle_) r4w_ringbuffer_free(handle_);
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    /**
     * @brief Get buffer capacity
     */
    size_t capacity() const { return r4w_ringbuffer_capacity(handle_); }

    /**
     * @brief Get current number of elements
     */
    size_t size() const { return r4w_ringbuffer_len(handle_); }

    /**
     * @brief Check if buffer is empty
     */
    bool empty() const { return r4w_ringbuffer_is_empty(handle_); }

    /**
     * @brief Check if buffer is full
     */
    bool full() const { return r4w_ringbuffer_is_full(handle_); }

    /**
     * @brief Push a single element
     * @param value Element to push
     * @return true if successful, false if buffer is full
     */
    bool push(const T& value) {
        return r4w_ringbuffer_push(handle_, from_complex(value)) == R4W_ERROR_OK;
    }

    /**
     * @brief Pop a single element
     * @return Element if available, std::nullopt if buffer is empty
     */
    std::optional<T> pop() {
        r4w_complex_t out;
        if (r4w_ringbuffer_pop(handle_, &out) == R4W_ERROR_OK) {
            return to_complex(out);
        }
        return std::nullopt;
    }

    /**
     * @brief Push multiple elements
     * @param data Elements to push
     * @return Number of elements actually pushed
     */
    size_t push_slice(const std::vector<T>& data) {
        return r4w_ringbuffer_push_slice(handle_,
            reinterpret_cast<const r4w_complex_t*>(data.data()), data.size());
    }

    /**
     * @brief Pop multiple elements
     * @param count Maximum number to pop
     * @return Vector of popped elements
     */
    std::vector<T> pop_slice(size_t count) {
        std::vector<T> out(count);
        size_t popped = r4w_ringbuffer_pop_slice(handle_,
            reinterpret_cast<r4w_complex_t*>(out.data()), count);
        out.resize(popped);
        return out;
    }

    /**
     * @brief Clear all elements
     */
    void clear() { r4w_ringbuffer_clear(handle_); }

private:
    r4w_ringbuffer_t* handle_;
};

// ============================================================================
// Waveform (MF-032)
// ============================================================================

/**
 * @brief Waveform type enumeration
 */
enum class WaveformType {
    BPSK = 0,
    QPSK = 1,
    LoRa = 2,
};

/**
 * @brief Generic waveform for modulation/demodulation
 *
 * Supports BPSK, QPSK, and LoRa waveforms through factory methods.
 */
class Waveform {
public:
    /**
     * @brief Create a BPSK waveform
     * @param sample_rate Sample rate in Hz
     * @param symbol_rate Symbol rate in baud
     * @return Waveform instance
     */
    static Waveform bpsk(double sample_rate, double symbol_rate) {
        auto* handle = r4w_waveform_bpsk_new(sample_rate, symbol_rate);
        if (!handle) throw Error("Failed to create BPSK waveform");
        return Waveform(handle);
    }

    /**
     * @brief Create a QPSK waveform
     * @param sample_rate Sample rate in Hz
     * @param symbol_rate Symbol rate in baud
     * @return Waveform instance
     */
    static Waveform qpsk(double sample_rate, double symbol_rate) {
        auto* handle = r4w_waveform_qpsk_new(sample_rate, symbol_rate);
        if (!handle) throw Error("Failed to create QPSK waveform");
        return Waveform(handle);
    }

    /**
     * @brief Create a LoRa waveform
     * @param spreading_factor Spreading factor (5-12)
     * @param bandwidth Bandwidth in Hz (125000, 250000, 500000)
     * @param sample_rate Sample rate in Hz
     * @return Waveform instance
     */
    static Waveform lora(uint8_t spreading_factor, uint32_t bandwidth, double sample_rate) {
        auto* handle = r4w_waveform_lora_new(spreading_factor, bandwidth, sample_rate);
        if (!handle) throw Error("Failed to create LoRa waveform");
        return Waveform(handle);
    }

    ~Waveform() {
        if (handle_) {
            r4w_waveform_free(handle_);
        }
    }

    // Non-copyable, movable
    Waveform(const Waveform&) = delete;
    Waveform& operator=(const Waveform&) = delete;
    Waveform(Waveform&& other) noexcept : handle_(other.handle_) { other.handle_ = nullptr; }
    Waveform& operator=(Waveform&& other) noexcept {
        if (this != &other) {
            if (handle_) r4w_waveform_free(handle_);
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    /**
     * @brief Get waveform type
     */
    WaveformType type() const {
        return static_cast<WaveformType>(r4w_waveform_type(handle_));
    }

    /**
     * @brief Get waveform name
     */
    std::string name() const {
        const char* n = r4w_waveform_name(handle_);
        return n ? std::string(n) : "";
    }

    /**
     * @brief Get samples per symbol
     */
    size_t samples_per_symbol() const {
        return r4w_waveform_samples_per_symbol(handle_);
    }

    /**
     * @brief Modulate data into I/Q samples
     * @param data Input bytes to modulate
     * @return Vector of complex I/Q samples
     */
    std::vector<Complex> modulate(const std::vector<uint8_t>& data) const {
        size_t max_size = r4w_waveform_modulate_size(handle_, data.size());
        std::vector<Complex> samples(max_size);
        size_t written;
        auto err = r4w_waveform_modulate(handle_,
            data.data(), data.size(),
            reinterpret_cast<r4w_complex_t*>(samples.data()), max_size,
            &written);
        if (err != R4W_ERROR_OK) throw Error(err);
        samples.resize(written);
        return samples;
    }

    /**
     * @brief Demodulate I/Q samples back to data
     * @param samples Input I/Q samples
     * @return Demodulated data bytes
     */
    std::vector<uint8_t> demodulate(const std::vector<Complex>& samples) const {
        // Estimate output size (usually smaller than input)
        size_t max_size = samples.size() / samples_per_symbol() + 16;
        std::vector<uint8_t> data(max_size);
        size_t written;
        auto err = r4w_waveform_demodulate(handle_,
            reinterpret_cast<const r4w_complex_t*>(samples.data()), samples.size(),
            data.data(), max_size,
            &written);
        if (err != R4W_ERROR_OK) throw Error(err);
        data.resize(written);
        return data;
    }

private:
    explicit Waveform(r4w_waveform_t* handle) : handle_(handle) {}
    r4w_waveform_t* handle_;
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Generate a complex tone signal
 * @param frequency Tone frequency in Hz
 * @param sample_rate Sample rate in Hz
 * @param length Number of samples
 * @return Vector of complex samples
 */
inline std::vector<Complex> generate_tone(double frequency, double sample_rate, size_t length) {
    std::vector<Complex> samples(length);
    auto err = r4w_generate_tone(
        reinterpret_cast<r4w_complex_t*>(samples.data()),
        length, frequency, sample_rate);
    if (err != R4W_ERROR_OK) throw Error(err);
    return samples;
}

/**
 * @brief Add AWGN (Additive White Gaussian Noise) to a signal
 * @param samples Signal to add noise to (modified in-place)
 * @param snr_db Signal-to-noise ratio in dB
 */
inline void add_awgn(std::vector<Complex>& samples, double snr_db) {
    auto err = r4w_add_awgn(
        reinterpret_cast<r4w_complex_t*>(samples.data()),
        samples.size(), snr_db);
    if (err != R4W_ERROR_OK) throw Error(err);
}

/**
 * @brief Frequency shift a signal
 * @param samples Signal to shift (modified in-place)
 * @param frequency_offset Frequency shift in Hz
 * @param sample_rate Sample rate in Hz
 */
inline void frequency_shift(std::vector<Complex>& samples, double frequency_offset, double sample_rate) {
    auto err = r4w_frequency_shift(
        reinterpret_cast<r4w_complex_t*>(samples.data()),
        samples.size(), frequency_offset, sample_rate);
    if (err != R4W_ERROR_OK) throw Error(err);
}

} // namespace r4w

#endif // R4W_HPP
