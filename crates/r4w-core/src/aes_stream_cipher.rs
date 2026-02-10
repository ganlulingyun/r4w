//! AES-CTR mode stream cipher for secure radio payload confidentiality.
//!
//! **WARNING**: This is an EDUCATIONAL/SIMULATION implementation of AES-128 in CTR mode.
//! It is NOT intended for production cryptographic use. For real-world applications,
//! use a vetted cryptographic library such as `aes` + `ctr` from RustCrypto.
//!
//! This module provides:
//! - A standalone AES-128 block cipher (single-block encryption)
//! - AES-CTR stream cipher built on top of it
//! - A `StreamCipher` trait for generic stream encryption/decryption
//!
//! # Example
//!
//! ```rust
//! use r4w_core::aes_stream_cipher::{AesCtr, StreamCipher};
//!
//! let key = [0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
//!            0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c];
//! let nonce = [0x00u8; 8];
//!
//! let mut cipher = AesCtr::new(&key, &nonce);
//! let plaintext = b"Hello, R4W radio!";
//! let ciphertext = cipher.encrypt(plaintext);
//!
//! // CTR mode is symmetric: decrypting is the same operation with a reset counter
//! cipher.reset();
//! let recovered = cipher.decrypt(&ciphertext);
//! assert_eq!(&recovered, plaintext);
//! ```

// ---------------------------------------------------------------------------
// AES-128 constants
// ---------------------------------------------------------------------------

/// AES S-box substitution table (FIPS-197, Section 5.1.1).
#[rustfmt::skip]
const SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
];

/// AES round constants (FIPS-197, Section 5.2).
const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

// ---------------------------------------------------------------------------
// StreamCipher trait
// ---------------------------------------------------------------------------

/// Generic stream cipher trait for encrypt/decrypt operations.
///
/// CTR-mode ciphers are symmetric (encrypt == decrypt), but the trait
/// keeps both methods for API clarity and to support non-CTR ciphers
/// in the future.
pub trait StreamCipher {
    /// Encrypt `data` and return the ciphertext.
    fn encrypt(&mut self, data: &[u8]) -> Vec<u8>;

    /// Decrypt `data` and return the plaintext.
    fn decrypt(&mut self, data: &[u8]) -> Vec<u8>;

    /// Reset the cipher state (counter, IV, etc.) so the same keystream
    /// is produced again from the beginning.
    fn reset(&mut self);
}

// ---------------------------------------------------------------------------
// Internal AES-128 helpers
// ---------------------------------------------------------------------------

/// Multiply a byte by 2 in GF(2^8) with the AES irreducible polynomial.
#[inline]
fn xtime(a: u8) -> u8 {
    let shifted = (a as u16) << 1;
    if a & 0x80 != 0 {
        (shifted ^ 0x1b) as u8
    } else {
        shifted as u8
    }
}

/// Multiply two bytes in GF(2^8).
#[inline]
fn gmul(mut a: u8, mut b: u8) -> u8 {
    let mut result: u8 = 0;
    while b != 0 {
        if b & 1 != 0 {
            result ^= a;
        }
        a = xtime(a);
        b >>= 1;
    }
    result
}

/// SubBytes: apply S-box to every byte of the 4x4 state.
fn sub_bytes(state: &mut [u8; 16]) {
    for byte in state.iter_mut() {
        *byte = SBOX[*byte as usize];
    }
}

/// ShiftRows: cyclically shift rows of the 4x4 state (column-major order).
///
/// Row 0: no shift
/// Row 1: shift left by 1
/// Row 2: shift left by 2
/// Row 3: shift left by 3
fn shift_rows(state: &mut [u8; 16]) {
    // The state is stored column-major: state[row + 4*col]
    // Row 1
    let tmp = state[1];
    state[1] = state[5];
    state[5] = state[9];
    state[9] = state[13];
    state[13] = tmp;

    // Row 2
    let tmp0 = state[2];
    let tmp1 = state[6];
    state[2] = state[10];
    state[6] = state[14];
    state[10] = tmp0;
    state[14] = tmp1;

    // Row 3
    let tmp = state[15];
    state[15] = state[11];
    state[11] = state[7];
    state[7] = state[3];
    state[3] = tmp;
}

/// MixColumns: mix each column of the 4x4 state using GF(2^8) arithmetic.
fn mix_columns(state: &mut [u8; 16]) {
    for col in 0..4 {
        let i = col * 4;
        let s0 = state[i];
        let s1 = state[i + 1];
        let s2 = state[i + 2];
        let s3 = state[i + 3];

        state[i]     = gmul(2, s0) ^ gmul(3, s1) ^ s2 ^ s3;
        state[i + 1] = s0 ^ gmul(2, s1) ^ gmul(3, s2) ^ s3;
        state[i + 2] = s0 ^ s1 ^ gmul(2, s2) ^ gmul(3, s3);
        state[i + 3] = gmul(3, s0) ^ s1 ^ s2 ^ gmul(2, s3);
    }
}

/// AddRoundKey: XOR the state with a 16-byte round key.
fn add_round_key(state: &mut [u8; 16], round_key: &[u8]) {
    for (s, k) in state.iter_mut().zip(round_key.iter()) {
        *s ^= k;
    }
}

/// AES-128 key expansion: produce 11 round keys (176 bytes total) from a
/// 16-byte key.
fn key_expansion(key: &[u8; 16]) -> Vec<u8> {
    let mut expanded = vec![0u8; 176]; // 11 * 16
    expanded[..16].copy_from_slice(key);

    let mut i = 16; // byte index into expanded key
    let mut rcon_idx = 0;

    while i < 176 {
        let mut temp = [
            expanded[i - 4],
            expanded[i - 3],
            expanded[i - 2],
            expanded[i - 1],
        ];

        if i % 16 == 0 {
            // RotWord
            temp.rotate_left(1);
            // SubWord
            for t in temp.iter_mut() {
                *t = SBOX[*t as usize];
            }
            // XOR with Rcon
            temp[0] ^= RCON[rcon_idx];
            rcon_idx += 1;
        }

        for j in 0..4 {
            expanded[i + j] = expanded[i - 16 + j] ^ temp[j];
        }
        i += 4;
    }

    expanded
}

// ---------------------------------------------------------------------------
// Public AES-128 single-block encryption
// ---------------------------------------------------------------------------

/// Encrypt a single 16-byte block with AES-128.
///
/// This performs the standard 10-round AES-128 encryption as specified in
/// FIPS-197. The input and output are both 16-byte arrays.
///
/// # Arguments
/// * `plaintext` - 16-byte input block
/// * `key` - 16-byte AES-128 key
///
/// # Returns
/// 16-byte encrypted block
pub fn aes_128_encrypt_block(plaintext: &[u8; 16], key: &[u8; 16]) -> [u8; 16] {
    let round_keys = key_expansion(key);

    // Copy plaintext into state (column-major, same byte order as input)
    let mut state = *plaintext;

    // Initial round key addition
    add_round_key(&mut state, &round_keys[0..16]);

    // Rounds 1..9 (with MixColumns)
    for round in 1..10 {
        sub_bytes(&mut state);
        shift_rows(&mut state);
        mix_columns(&mut state);
        add_round_key(&mut state, &round_keys[round * 16..(round + 1) * 16]);
    }

    // Final round (no MixColumns)
    sub_bytes(&mut state);
    shift_rows(&mut state);
    add_round_key(&mut state, &round_keys[160..176]);

    state
}

// ---------------------------------------------------------------------------
// XOR helper
// ---------------------------------------------------------------------------

/// XOR two byte slices element-wise, returning a Vec of the same length as
/// the shorter input.
///
/// # Arguments
/// * `a` - first byte slice
/// * `b` - second byte slice
///
/// # Returns
/// A `Vec<u8>` where each byte is `a[i] ^ b[i]`.
pub fn xor_bytes(a: &[u8], b: &[u8]) -> Vec<u8> {
    a.iter().zip(b.iter()).map(|(x, y)| x ^ y).collect()
}

// ---------------------------------------------------------------------------
// AesCtr
// ---------------------------------------------------------------------------

/// AES-128 in CTR (Counter) mode.
///
/// The counter block is formed as `nonce (8 bytes) || counter (8 bytes, big-endian)`.
/// Each call to `encrypt` / `decrypt` / `keystream` advances the counter.
/// Use `reset()` to rewind the counter to zero.
pub struct AesCtr {
    key: [u8; 16],
    nonce: [u8; 8],
    counter: u64,
    /// Leftover keystream bytes from a partially consumed block.
    buffer: Vec<u8>,
}

impl AesCtr {
    /// Create a new AES-128 CTR cipher.
    ///
    /// # Arguments
    /// * `key` - 16-byte AES-128 key
    /// * `nonce` - 8-byte nonce (the remaining 8 bytes of the counter block
    ///   are the big-endian block counter starting at 0)
    pub fn new(key: &[u8; 16], nonce: &[u8; 8]) -> Self {
        Self {
            key: *key,
            nonce: *nonce,
            counter: 0,
            buffer: Vec::new(),
        }
    }

    /// Encrypt `plaintext` by XOR-ing with the AES-CTR keystream.
    ///
    /// Because CTR mode XORs plaintext with a keystream, encryption and
    /// decryption are the same operation.
    pub fn encrypt(&mut self, plaintext: &[u8]) -> Vec<u8> {
        let ks = self.keystream(plaintext.len());
        xor_bytes(plaintext, &ks)
    }

    /// Decrypt `ciphertext` by XOR-ing with the AES-CTR keystream.
    ///
    /// Identical to [`encrypt`](Self::encrypt) because CTR mode is symmetric.
    pub fn decrypt(&mut self, ciphertext: &[u8]) -> Vec<u8> {
        let ks = self.keystream(ciphertext.len());
        xor_bytes(ciphertext, &ks)
    }

    /// Reset the counter to zero, so subsequent calls reproduce the same
    /// keystream from the start.
    pub fn reset(&mut self) {
        self.counter = 0;
        self.buffer.clear();
    }

    /// Generate `len` bytes of raw keystream.
    ///
    /// The keystream is produced by encrypting successive counter blocks
    /// with AES-128. Any leftover bytes from a previous partial block are
    /// consumed first.
    pub fn keystream(&mut self, len: usize) -> Vec<u8> {
        let mut out = Vec::with_capacity(len);

        // Drain any leftover buffered keystream bytes first
        if !self.buffer.is_empty() {
            let take = len.min(self.buffer.len());
            out.extend_from_slice(&self.buffer[..take]);
            self.buffer = self.buffer[take..].to_vec();
            if out.len() == len {
                return out;
            }
        }

        // Generate full blocks until we have enough
        while out.len() < len {
            let mut counter_block = [0u8; 16];
            counter_block[..8].copy_from_slice(&self.nonce);
            counter_block[8..16].copy_from_slice(&self.counter.to_be_bytes());
            self.counter += 1;

            let encrypted = aes_128_encrypt_block(&counter_block, &self.key);
            let remaining = len - out.len();

            if remaining >= 16 {
                out.extend_from_slice(&encrypted);
            } else {
                out.extend_from_slice(&encrypted[..remaining]);
                // Buffer the unused tail for the next call
                self.buffer = encrypted[remaining..].to_vec();
            }
        }

        out
    }
}

impl StreamCipher for AesCtr {
    fn encrypt(&mut self, data: &[u8]) -> Vec<u8> {
        AesCtr::encrypt(self, data)
    }

    fn decrypt(&mut self, data: &[u8]) -> Vec<u8> {
        AesCtr::decrypt(self, data)
    }

    fn reset(&mut self) {
        AesCtr::reset(self);
    }
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

/// Create an `AesCtr` instance with an all-zero key and nonce.
///
/// **For testing only.** Using an all-zero key provides no security.
pub fn aes_ctr_zero_key() -> AesCtr {
    AesCtr::new(&[0u8; 16], &[0u8; 8])
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encrypt_then_decrypt_recovers_plaintext() {
        let key = [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
            0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
        ];
        let nonce = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
        let plaintext = b"The quick brown fox jumps over the lazy dog!";

        let mut enc = AesCtr::new(&key, &nonce);
        let ciphertext = enc.encrypt(plaintext);

        let mut dec = AesCtr::new(&key, &nonce);
        let recovered = dec.decrypt(&ciphertext);

        assert_eq!(&recovered[..], &plaintext[..]);
    }

    #[test]
    fn test_ctr_encrypt_equals_decrypt() {
        // In CTR mode, encrypt and decrypt are the same XOR operation.
        let key = [0xAA; 16];
        let nonce = [0xBB; 8];
        let data = b"symmetric operation test";

        let mut c1 = AesCtr::new(&key, &nonce);
        let mut c2 = AesCtr::new(&key, &nonce);

        let out1 = c1.encrypt(data);
        let out2 = c2.decrypt(data);

        assert_eq!(out1, out2, "encrypt and decrypt must produce identical output in CTR mode");
    }

    #[test]
    fn test_keystream_changes_with_different_nonces() {
        let key = [0x00; 16];
        let nonce_a = [0x00; 8];
        let nonce_b = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];

        let mut a = AesCtr::new(&key, &nonce_a);
        let mut b = AesCtr::new(&key, &nonce_b);

        let ks_a = a.keystream(32);
        let ks_b = b.keystream(32);

        assert_ne!(ks_a, ks_b, "different nonces must produce different keystreams");
    }

    #[test]
    fn test_keystream_changes_with_different_keys() {
        let key_a = [0x00; 16];
        let key_b = [0x01; 16];
        let nonce = [0x00; 8];

        let mut a = AesCtr::new(&key_a, &nonce);
        let mut b = AesCtr::new(&key_b, &nonce);

        let ks_a = a.keystream(32);
        let ks_b = b.keystream(32);

        assert_ne!(ks_a, ks_b, "different keys must produce different keystreams");
    }

    #[test]
    fn test_reset_produces_same_keystream() {
        let key = [0x42; 16];
        let nonce = [0x13; 8];

        let mut cipher = AesCtr::new(&key, &nonce);

        let ks1 = cipher.keystream(48);
        cipher.reset();
        let ks2 = cipher.keystream(48);

        assert_eq!(ks1, ks2, "reset must reproduce the same keystream");
    }

    #[test]
    fn test_aes_128_single_block_fips197() {
        // FIPS-197 Appendix B test vector.
        let key: [u8; 16] = [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
            0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
        ];
        let plaintext: [u8; 16] = [
            0x32, 0x43, 0xf6, 0xa8, 0x88, 0x5a, 0x30, 0x8d,
            0x31, 0x31, 0x98, 0xa2, 0xe0, 0x37, 0x07, 0x34,
        ];
        let expected: [u8; 16] = [
            0x39, 0x25, 0x84, 0x1d, 0x02, 0xdc, 0x09, 0xfb,
            0xdc, 0x11, 0x85, 0x97, 0x19, 0x6a, 0x0b, 0x32,
        ];

        let result = aes_128_encrypt_block(&plaintext, &key);
        assert_eq!(result, expected, "AES-128 block encryption must match FIPS-197 Appendix B");
    }

    #[test]
    fn test_xor_bytes_correctness() {
        let a = [0xFF, 0x00, 0xAA, 0x55];
        let b = [0x0F, 0xF0, 0x55, 0xAA];
        let result = xor_bytes(&a, &b);
        assert_eq!(result, vec![0xF0, 0xF0, 0xFF, 0xFF]);

        // XOR with self yields zeros
        let self_xor = xor_bytes(&a, &a);
        assert_eq!(self_xor, vec![0x00, 0x00, 0x00, 0x00]);

        // Different lengths: result is truncated to shorter
        let short = [0xFF, 0x00];
        let long = [0x0F, 0xF0, 0xAA];
        let truncated = xor_bytes(&short, &long);
        assert_eq!(truncated, vec![0xF0, 0xF0]);
    }

    #[test]
    fn test_empty_input_produces_empty_output() {
        let key = [0x12; 16];
        let nonce = [0x34; 8];

        let mut cipher = AesCtr::new(&key, &nonce);

        let enc = cipher.encrypt(b"");
        assert!(enc.is_empty(), "encrypting empty input must produce empty output");

        let dec = cipher.decrypt(b"");
        assert!(dec.is_empty(), "decrypting empty input must produce empty output");

        let ks = cipher.keystream(0);
        assert!(ks.is_empty(), "zero-length keystream must be empty");
    }

    #[test]
    fn test_multi_block_encryption() {
        // Test with data spanning multiple 16-byte AES blocks
        let key = [0x01; 16];
        let nonce = [0x02; 8];
        let plaintext = vec![0xAB; 100]; // 100 bytes = 6 full blocks + 4 bytes

        let mut enc = AesCtr::new(&key, &nonce);
        let ciphertext = enc.encrypt(&plaintext);
        assert_eq!(ciphertext.len(), 100);

        // Ciphertext must differ from plaintext (with overwhelming probability)
        assert_ne!(ciphertext, plaintext);

        // Decrypt must recover the original
        let mut dec = AesCtr::new(&key, &nonce);
        let recovered = dec.decrypt(&ciphertext);
        assert_eq!(recovered, plaintext);
    }

    #[test]
    fn test_zero_key_factory() {
        let mut cipher = aes_ctr_zero_key();

        // The factory should produce a usable cipher
        let plaintext = b"zero-key test data";
        let ciphertext = cipher.encrypt(plaintext);
        assert_eq!(ciphertext.len(), plaintext.len());

        // It must actually encrypt (not passthrough)
        assert_ne!(&ciphertext[..], &plaintext[..]);

        // Must be recoverable
        cipher.reset();
        let recovered = cipher.decrypt(&ciphertext);
        assert_eq!(&recovered[..], &plaintext[..]);

        // Verify it's truly all-zero key/nonce by comparing with a manually
        // constructed instance
        let mut manual = AesCtr::new(&[0u8; 16], &[0u8; 8]);
        let ks_factory = {
            cipher.reset();
            cipher.keystream(32)
        };
        let ks_manual = manual.keystream(32);
        assert_eq!(ks_factory, ks_manual);
    }
}
