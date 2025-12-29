/**
 * R4W FFT Demo - Using R4W DSP functions from C
 *
 * This example demonstrates:
 * - FFT operations (forward, inverse, peak finding)
 * - Complex number operations
 * - Chirp generation (LoRa-style)
 * - Lock-free ring buffer for streaming
 *
 * Build:
 *   # First, build the R4W library
 *   cargo build --release -p r4w-ffi
 *
 *   # Then compile this program
 *   gcc -o fft_demo fft_demo.c -I../../crates/r4w-ffi/include \
 *       -L../../target/release -lr4w -lm -lpthread
 *
 *   # Set library path and run
 *   LD_LIBRARY_PATH=../../target/release ./fft_demo
 *
 * Or use the CMakeLists.txt in this directory for a more portable build.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Include the R4W header directly as C */
#ifdef __cplusplus
extern "C" {
#endif

/* R4W types and functions - C-compatible subset of r4w.h */
typedef enum {
    R4W_ERROR_OK = 0,
    R4W_ERROR_NULL_POINTER = 1,
    R4W_ERROR_INVALID_SIZE = 2,
    R4W_ERROR_BUFFER_FULL = 3,
    R4W_ERROR_BUFFER_EMPTY = 4,
    R4W_ERROR_INVALID_PARAMETER = 5,
    R4W_ERROR_ALLOCATION_FAILED = 6,
    R4W_ERROR_NOT_SUPPORTED = 7,
} r4w_error_t;

typedef struct {
    double re;
    double im;
} r4w_complex_t;

/* Opaque types */
typedef struct R4wFftProcessor r4w_fft_t;
typedef struct R4wChirpGenerator r4w_chirp_t;
typedef struct R4wRingBuffer r4w_ringbuffer_t;

/* Function declarations */
extern const char* r4w_version(void);
extern r4w_error_t r4w_init(void);

/* Complex operations */
extern r4w_complex_t r4w_complex_new(double re, double im);
extern r4w_complex_t r4w_complex_from_polar(double magnitude, double phase);
extern double r4w_complex_magnitude(r4w_complex_t c);
extern double r4w_complex_phase(r4w_complex_t c);
extern double r4w_complex_power(r4w_complex_t c);
extern r4w_complex_t r4w_complex_conj(r4w_complex_t c);
extern r4w_complex_t r4w_complex_mul(r4w_complex_t a, r4w_complex_t b);
extern r4w_complex_t r4w_complex_add(r4w_complex_t a, r4w_complex_t b);
extern double r4w_complex_average_power(const r4w_complex_t* buffer, size_t len);
extern r4w_error_t r4w_complex_normalize(r4w_complex_t* buffer, size_t len);

/* FFT operations */
extern r4w_fft_t* r4w_fft_new(size_t size);
extern void r4w_fft_free(r4w_fft_t* fft);
extern size_t r4w_fft_size(const r4w_fft_t* fft);
extern r4w_error_t r4w_fft_forward(r4w_fft_t* fft, r4w_complex_t* buffer, size_t len);
extern r4w_error_t r4w_fft_inverse(r4w_fft_t* fft, r4w_complex_t* buffer, size_t len);
extern r4w_error_t r4w_fft_find_peak(const r4w_complex_t* buffer, size_t len,
                                      size_t* out_bin, double* out_magnitude, double* out_phase);
extern r4w_error_t r4w_fft_find_peak_interpolated(const r4w_complex_t* buffer, size_t len,
                                                   double* out_bin, double* out_magnitude);
extern r4w_error_t r4w_fft_magnitude_spectrum(const r4w_complex_t* input, double* output, size_t len);
extern r4w_error_t r4w_fft_power_spectrum_db(const r4w_complex_t* input, double* output, size_t len);

/* Chirp generation */
extern r4w_chirp_t* r4w_chirp_new(unsigned char spreading_factor, unsigned int bandwidth, size_t oversample);
extern void r4w_chirp_free(r4w_chirp_t* chirp);
extern size_t r4w_chirp_samples_per_symbol(const r4w_chirp_t* chirp);
extern double r4w_chirp_bandwidth(const r4w_chirp_t* chirp);
extern r4w_error_t r4w_chirp_generate_upchirp(const r4w_chirp_t* chirp, r4w_complex_t* buffer, size_t len);
extern r4w_error_t r4w_chirp_generate_downchirp(const r4w_chirp_t* chirp, r4w_complex_t* buffer, size_t len);
extern r4w_error_t r4w_chirp_modulate_symbol(const r4w_chirp_t* chirp, unsigned short symbol,
                                              r4w_complex_t* buffer, size_t len);

/* Ring buffer */
extern r4w_ringbuffer_t* r4w_ringbuffer_new(size_t capacity);
extern void r4w_ringbuffer_free(r4w_ringbuffer_t* rb);
extern size_t r4w_ringbuffer_capacity(const r4w_ringbuffer_t* rb);
extern size_t r4w_ringbuffer_len(const r4w_ringbuffer_t* rb);
extern int r4w_ringbuffer_is_empty(const r4w_ringbuffer_t* rb);
extern int r4w_ringbuffer_is_full(const r4w_ringbuffer_t* rb);
extern r4w_error_t r4w_ringbuffer_push(const r4w_ringbuffer_t* rb, r4w_complex_t sample);
extern r4w_error_t r4w_ringbuffer_pop(const r4w_ringbuffer_t* rb, r4w_complex_t* out);
extern size_t r4w_ringbuffer_push_slice(const r4w_ringbuffer_t* rb, const r4w_complex_t* buffer, size_t len);
extern size_t r4w_ringbuffer_pop_slice(const r4w_ringbuffer_t* rb, r4w_complex_t* buffer, size_t len);

/* Signal generation */
extern r4w_error_t r4w_generate_tone(double frequency, double sample_rate,
                                      r4w_complex_t* buffer, size_t len);
extern r4w_error_t r4w_add_awgn(r4w_complex_t* buffer, size_t len, double snr_db);
extern r4w_error_t r4w_frequency_shift(r4w_complex_t* buffer, size_t len,
                                        double frequency_offset, double sample_rate);

#ifdef __cplusplus
}
#endif

#define PI 3.14159265358979323846

/* Demo 1: Basic FFT with a single tone */
void demo_fft_single_tone(void) {
    printf("\n=== Demo 1: FFT Single Tone Detection ===\n");

    const size_t N = 1024;
    const double sample_rate = 1000.0;  /* 1 kHz */
    const double tone_freq = 50.0;      /* 50 Hz tone */

    /* Create FFT processor */
    r4w_fft_t* fft = r4w_fft_new(N);
    if (!fft) {
        fprintf(stderr, "Failed to create FFT processor\n");
        return;
    }

    /* Allocate buffer */
    r4w_complex_t* buffer = (r4w_complex_t*)malloc(N * sizeof(r4w_complex_t));
    if (!buffer) {
        r4w_fft_free(fft);
        return;
    }

    /* Generate a pure tone using R4W function */
    r4w_error_t err = r4w_generate_tone(tone_freq, sample_rate, buffer, N);
    if (err != R4W_ERROR_OK) {
        fprintf(stderr, "Failed to generate tone\n");
        free(buffer);
        r4w_fft_free(fft);
        return;
    }

    printf("Generated %.1f Hz tone at %.1f Hz sample rate\n", tone_freq, sample_rate);
    printf("Signal power: %.4f\n", r4w_complex_average_power(buffer, N));

    /* Perform FFT */
    err = r4w_fft_forward(fft, buffer, N);
    if (err != R4W_ERROR_OK) {
        fprintf(stderr, "FFT failed\n");
        free(buffer);
        r4w_fft_free(fft);
        return;
    }

    /* Find the peak */
    size_t peak_bin;
    double peak_magnitude, peak_phase;
    r4w_fft_find_peak(buffer, N, &peak_bin, &peak_magnitude, &peak_phase);

    double detected_freq = (double)peak_bin * sample_rate / (double)N;
    printf("Peak at bin %zu (%.1f Hz), magnitude=%.2f, phase=%.2f rad\n",
           peak_bin, detected_freq, peak_magnitude, peak_phase);

    /* Try interpolated peak for sub-bin accuracy */
    double interp_bin, interp_mag;
    r4w_fft_find_peak_interpolated(buffer, N, &interp_bin, &interp_mag);
    double interp_freq = interp_bin * sample_rate / (double)N;
    printf("Interpolated: bin=%.3f (%.3f Hz), magnitude=%.2f\n",
           interp_bin, interp_freq, interp_mag);

    /* Cleanup */
    free(buffer);
    r4w_fft_free(fft);
}

/* Demo 2: FFT with noise and SNR estimation */
void demo_fft_with_noise(void) {
    printf("\n=== Demo 2: FFT with AWGN ===\n");

    const size_t N = 1024;
    const double sample_rate = 1000.0;
    const double tone_freq = 100.0;
    const double snr_db = 10.0;

    r4w_fft_t* fft = r4w_fft_new(N);
    r4w_complex_t* buffer = (r4w_complex_t*)malloc(N * sizeof(r4w_complex_t));

    if (!fft || !buffer) {
        fprintf(stderr, "Allocation failed\n");
        if (fft) r4w_fft_free(fft);
        if (buffer) free(buffer);
        return;
    }

    /* Generate clean tone */
    r4w_generate_tone(tone_freq, sample_rate, buffer, N);
    printf("Clean signal power: %.4f\n", r4w_complex_average_power(buffer, N));

    /* Add noise */
    r4w_add_awgn(buffer, N, snr_db);
    printf("After adding %.1f dB SNR noise, power: %.4f\n",
           snr_db, r4w_complex_average_power(buffer, N));

    /* FFT and find peak */
    r4w_fft_forward(fft, buffer, N);

    size_t peak_bin;
    double peak_mag, peak_phase;
    r4w_fft_find_peak(buffer, N, &peak_bin, &peak_mag, &peak_phase);

    printf("Peak at bin %zu (%.1f Hz), magnitude=%.2f\n",
           peak_bin, (double)peak_bin * sample_rate / N, peak_mag);

    free(buffer);
    r4w_fft_free(fft);
}

/* Demo 3: LoRa chirp generation and demodulation */
void demo_chirp_generation(void) {
    printf("\n=== Demo 3: LoRa Chirp Generation ===\n");

    /* Create chirp generator: SF=7, BW=125kHz, oversample=1 */
    r4w_chirp_t* chirp = r4w_chirp_new(7, 125000, 1);
    if (!chirp) {
        fprintf(stderr, "Failed to create chirp generator\n");
        return;
    }

    size_t samples_per_symbol = r4w_chirp_samples_per_symbol(chirp);
    double bandwidth = r4w_chirp_bandwidth(chirp);

    printf("Spreading Factor: 7\n");
    printf("Bandwidth: %.0f Hz\n", bandwidth);
    printf("Samples per symbol: %zu\n", samples_per_symbol);
    printf("Bits per symbol: 7 (SF)\n");
    printf("Symbol rate: %.1f symbols/sec\n", bandwidth / samples_per_symbol);

    /* Allocate buffer for chirp */
    r4w_complex_t* chirp_buf = (r4w_complex_t*)malloc(samples_per_symbol * sizeof(r4w_complex_t));
    if (!chirp_buf) {
        r4w_chirp_free(chirp);
        return;
    }

    /* Generate base upchirp */
    r4w_chirp_generate_upchirp(chirp, chirp_buf, samples_per_symbol);
    printf("\nGenerated base upchirp:\n");
    printf("  First 5 samples: ");
    for (size_t i = 0; i < 5; i++) {
        printf("(%.3f,%.3f) ", chirp_buf[i].re, chirp_buf[i].im);
    }
    printf("\n");

    /* Verify unit magnitude */
    double min_mag = 1e10, max_mag = 0;
    for (size_t i = 0; i < samples_per_symbol; i++) {
        double mag = r4w_complex_magnitude(chirp_buf[i]);
        if (mag < min_mag) min_mag = mag;
        if (mag > max_mag) max_mag = mag;
    }
    printf("  Magnitude range: [%.6f, %.6f] (should be ~1.0)\n", min_mag, max_mag);

    /* Generate a modulated symbol (symbol 64 = halfway through) */
    unsigned short symbol = 64;
    r4w_chirp_modulate_symbol(chirp, symbol, chirp_buf, samples_per_symbol);
    printf("\nModulated symbol %u:\n", symbol);
    printf("  First 5 samples: ");
    for (size_t i = 0; i < 5; i++) {
        printf("(%.3f,%.3f) ", chirp_buf[i].re, chirp_buf[i].im);
    }
    printf("\n");

    /* Demonstrate demodulation concept:
     * Multiply received chirp by downchirp, then FFT.
     * Peak bin = transmitted symbol */
    r4w_complex_t* downchirp = (r4w_complex_t*)malloc(samples_per_symbol * sizeof(r4w_complex_t));
    r4w_complex_t* product = (r4w_complex_t*)malloc(samples_per_symbol * sizeof(r4w_complex_t));

    if (downchirp && product) {
        r4w_chirp_generate_downchirp(chirp, downchirp, samples_per_symbol);

        /* Multiply: received * conjugate(downchirp) */
        for (size_t i = 0; i < samples_per_symbol; i++) {
            r4w_complex_t conj_down = r4w_complex_conj(downchirp[i]);
            product[i] = r4w_complex_mul(chirp_buf[i], conj_down);
        }

        /* FFT to find symbol */
        r4w_fft_t* fft = r4w_fft_new(samples_per_symbol);
        if (fft) {
            r4w_fft_forward(fft, product, samples_per_symbol);

            size_t detected_bin;
            double mag, phase;
            r4w_fft_find_peak(product, samples_per_symbol, &detected_bin, &mag, &phase);

            printf("\nDemodulation result:\n");
            printf("  Transmitted symbol: %u\n", symbol);
            printf("  Detected bin: %zu\n", detected_bin);
            printf("  Match: %s\n", detected_bin == symbol ? "YES" : "NO");

            r4w_fft_free(fft);
        }
    }

    if (downchirp) free(downchirp);
    if (product) free(product);
    free(chirp_buf);
    r4w_chirp_free(chirp);
}

/* Demo 4: Lock-free ring buffer for streaming */
void demo_ring_buffer(void) {
    printf("\n=== Demo 4: Lock-Free Ring Buffer ===\n");

    const size_t capacity = 1024;
    r4w_ringbuffer_t* rb = r4w_ringbuffer_new(capacity);
    if (!rb) {
        fprintf(stderr, "Failed to create ring buffer\n");
        return;
    }

    printf("Created ring buffer with capacity %zu\n", r4w_ringbuffer_capacity(rb));
    printf("Initial state: empty=%d, len=%zu\n",
           r4w_ringbuffer_is_empty(rb), r4w_ringbuffer_len(rb));

    /* Push some samples */
    const size_t chunk_size = 256;
    r4w_complex_t* data = (r4w_complex_t*)malloc(chunk_size * sizeof(r4w_complex_t));
    for (size_t i = 0; i < chunk_size; i++) {
        data[i] = r4w_complex_new((double)i, (double)i * 2);
    }

    size_t pushed = r4w_ringbuffer_push_slice(rb, data, chunk_size);
    printf("Pushed %zu samples, buffer len=%zu\n", pushed, r4w_ringbuffer_len(rb));

    /* Pop some samples */
    r4w_complex_t* out = (r4w_complex_t*)malloc(128 * sizeof(r4w_complex_t));
    size_t popped = r4w_ringbuffer_pop_slice(rb, out, 128);
    printf("Popped %zu samples, buffer len=%zu\n", popped, r4w_ringbuffer_len(rb));

    /* Verify data integrity */
    int verified = 1;
    for (size_t i = 0; i < popped; i++) {
        if (out[i].re != (double)i || out[i].im != (double)i * 2) {
            verified = 0;
            break;
        }
    }
    printf("Data integrity: %s\n", verified ? "VERIFIED" : "FAILED");

    free(data);
    free(out);
    r4w_ringbuffer_free(rb);
}

/* Demo 5: Complex number operations */
void demo_complex_operations(void) {
    printf("\n=== Demo 5: Complex Number Operations ===\n");

    /* Create complex numbers */
    r4w_complex_t a = r4w_complex_new(3.0, 4.0);
    r4w_complex_t b = r4w_complex_new(1.0, 2.0);

    printf("a = %.1f + %.1fi\n", a.re, a.im);
    printf("b = %.1f + %.1fi\n", b.re, b.im);

    /* Magnitude and phase */
    printf("  |a| = %.4f (expected 5.0)\n", r4w_complex_magnitude(a));
    printf("  arg(a) = %.4f rad (expected %.4f)\n",
           r4w_complex_phase(a), atan2(4.0, 3.0));

    /* From polar */
    r4w_complex_t c = r4w_complex_from_polar(1.0, PI / 4.0);
    printf("  From polar(1, pi/4): %.4f + %.4fi\n", c.re, c.im);

    /* Operations */
    r4w_complex_t sum = r4w_complex_add(a, b);
    printf("  a + b = %.1f + %.1fi\n", sum.re, sum.im);

    r4w_complex_t prod = r4w_complex_mul(a, b);
    printf("  a * b = %.1f + %.1fi\n", prod.re, prod.im);

    r4w_complex_t conj = r4w_complex_conj(a);
    printf("  conj(a) = %.1f + %.1fi\n", conj.re, conj.im);

    /* Power */
    printf("  |a|^2 = %.4f\n", r4w_complex_power(a));
}

int main(int argc, char* argv[]) {
    printf("R4W FFI Demo - Version %s\n", r4w_version());
    printf("========================================\n");

    /* Initialize the library */
    r4w_error_t err = r4w_init();
    if (err != R4W_ERROR_OK) {
        fprintf(stderr, "Failed to initialize R4W library\n");
        return 1;
    }

    /* Run all demos */
    demo_complex_operations();
    demo_fft_single_tone();
    demo_fft_with_noise();
    demo_chirp_generation();
    demo_ring_buffer();

    printf("\n========================================\n");
    printf("All demos completed successfully!\n");

    return 0;
}
