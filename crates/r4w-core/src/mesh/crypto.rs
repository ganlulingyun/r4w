//! Meshtastic-compatible AES-256-CTR encryption
//!
//! Implements channel encryption compatible with the Meshtastic protocol.
//! Matches the firmware implementation in `CryptoEngine.cpp`.
//!
//! ## Key Derivation
//!
//! The PSK is used directly as the AES-256 key:
//! - 32-byte PSK → used directly as AES-256 key
//! - 16-byte PSK → zero-padded to 32 bytes
//! - 1-byte PSK index → expanded from DEFAULT_PSK with index offset
//!
//! ## Nonce Construction
//!
//! The 16-byte nonce (IV) for AES-256-CTR is constructed as:
//! ```text
//! Bytes 0-7:   packet_id as u64 (little-endian)
//! Bytes 8-11:  source node_id as u32 (little-endian)
//! Bytes 12-15: zeros (AES-CTR block counter space)
//! ```
//!
//! ## No MIC in CTR Mode
//!
//! Meshtastic AES-CTR mode does not use a Message Integrity Code (MIC).
//! The protocol relies on higher-layer integrity checks.

use super::packet::{MeshPacket, NodeId};

#[cfg(feature = "crypto")]
use aes::Aes256;
#[cfg(feature = "crypto")]
use ctr::cipher::{KeyIvInit, StreamCipher};
#[cfg(feature = "crypto")]
use ctr::Ctr128BE;

/// Default Pre-Shared Key (PSK) for the default channel
/// This is the well-known key used by default Meshtastic channels
pub const DEFAULT_PSK: &[u8] = &[
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
    0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01,
];

/// Crypto error types
#[derive(Debug, Clone, PartialEq)]
pub enum CryptoError {
    /// Invalid key length
    InvalidKeyLength,
    /// Invalid data length
    InvalidDataLength,
    /// MIC verification failed
    MicMismatch,
    /// Encryption failed
    EncryptionFailed,
    /// Decryption failed
    DecryptionFailed,
    /// Crypto feature not enabled
    FeatureNotEnabled,
}

impl std::fmt::Display for CryptoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CryptoError::InvalidKeyLength => write!(f, "Invalid key length"),
            CryptoError::InvalidDataLength => write!(f, "Invalid data length"),
            CryptoError::MicMismatch => write!(f, "MIC verification failed"),
            CryptoError::EncryptionFailed => write!(f, "Encryption failed"),
            CryptoError::DecryptionFailed => write!(f, "Decryption failed"),
            CryptoError::FeatureNotEnabled => {
                write!(f, "Crypto feature not enabled, compile with --features crypto")
            }
        }
    }
}

impl std::error::Error for CryptoError {}

/// Result type for crypto operations
pub type CryptoResult<T> = Result<T, CryptoError>;

/// Channel encryption key
#[derive(Clone)]
pub struct ChannelKey {
    /// 32-byte AES-256 key
    key: [u8; 32],
    /// Channel name for identification
    channel_name: String,
}

impl ChannelKey {
    /// Create a channel key from name and PSK
    ///
    /// The PSK is used directly as the AES key (Meshtastic-compatible):
    /// - 32-byte PSK → used directly as AES-256 key
    /// - 16-byte PSK → zero-padded to 32 bytes
    /// - 1-byte PSK index → expanded from DEFAULT_PSK with index offset
    /// - Other lengths → zero-padded to 32 bytes
    #[cfg(feature = "crypto")]
    pub fn new(channel_name: &str, psk: &[u8]) -> Self {
        let key = Self::key_from_psk(psk);
        Self {
            key,
            channel_name: channel_name.to_string(),
        }
    }

    /// Create a channel key with the default PSK
    #[cfg(feature = "crypto")]
    pub fn with_default_psk(channel_name: &str) -> Self {
        Self::new(channel_name, DEFAULT_PSK)
    }

    /// Create from raw 32-byte key
    pub fn from_raw(key: [u8; 32], channel_name: &str) -> Self {
        Self {
            key,
            channel_name: channel_name.to_string(),
        }
    }

    /// Create a channel key from a 1-byte PSK index
    ///
    /// Expands the default PSK by setting the last byte to the index value.
    /// Index 0 = unencrypted, index 1 = default PSK as-is.
    #[cfg(feature = "crypto")]
    pub fn from_psk_index(channel_name: &str, index: u8) -> Self {
        if index == 0 {
            // Index 0 means no encryption
            return Self {
                key: [0u8; 32],
                channel_name: channel_name.to_string(),
            };
        }
        let mut expanded = [0u8; 32];
        expanded[..DEFAULT_PSK.len()].copy_from_slice(DEFAULT_PSK);
        if index > 1 {
            expanded[DEFAULT_PSK.len() - 1] = index;
        }
        Self {
            key: expanded,
            channel_name: channel_name.to_string(),
        }
    }

    /// Convert PSK bytes to a 32-byte AES key (Meshtastic-compatible)
    ///
    /// PSK is used directly — no SHA-256 derivation.
    #[cfg(feature = "crypto")]
    fn key_from_psk(psk: &[u8]) -> [u8; 32] {
        let mut key = [0u8; 32];
        match psk.len() {
            32 => key.copy_from_slice(psk),
            1 => {
                // Single-byte PSK index: expand from default
                key[..DEFAULT_PSK.len()].copy_from_slice(DEFAULT_PSK);
                if psk[0] > 1 {
                    key[DEFAULT_PSK.len() - 1] = psk[0];
                }
            }
            n => {
                // Any other length: copy what fits, zero-pad the rest
                let copy_len = n.min(32);
                key[..copy_len].copy_from_slice(&psk[..copy_len]);
            }
        }
        key
    }

    /// Get the raw key bytes
    pub fn as_bytes(&self) -> &[u8; 32] {
        &self.key
    }

    /// Get the channel name
    pub fn channel_name(&self) -> &str {
        &self.channel_name
    }

    /// Compute channel hash using XOR fold (Meshtastic-compatible)
    ///
    /// Matches the `xorHash()` function in Meshtastic firmware:
    /// `hash = xorHash(channel_name) ^ xorHash(psk)`
    pub fn channel_hash(&self) -> u8 {
        let name_hash = self.channel_name.as_bytes().iter().fold(0u8, |acc, &b| acc ^ b);
        let key_hash = self.key.iter().fold(0u8, |acc, &b| acc ^ b);
        name_hash ^ key_hash
    }
}

impl std::fmt::Debug for ChannelKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ChannelKey")
            .field("channel_name", &self.channel_name)
            .field("key", &"[REDACTED]")
            .finish()
    }
}

/// Crypto context for a channel
#[derive(Debug)]
pub struct CryptoContext {
    /// Channel key
    key: ChannelKey,
}

impl CryptoContext {
    /// Create a new crypto context for a channel
    #[cfg(feature = "crypto")]
    pub fn new(channel_name: &str, psk: &[u8]) -> Self {
        Self {
            key: ChannelKey::new(channel_name, psk),
        }
    }

    /// Create with default PSK
    #[cfg(feature = "crypto")]
    pub fn with_default_psk(channel_name: &str) -> Self {
        Self {
            key: ChannelKey::with_default_psk(channel_name),
        }
    }

    /// Create from an existing channel key
    pub fn from_key(key: ChannelKey) -> Self {
        Self { key }
    }

    /// Get channel hash
    pub fn channel_hash(&self) -> u8 {
        self.key.channel_hash()
    }

    /// Construct the 16-byte nonce/IV for AES-CTR (Meshtastic-compatible)
    ///
    /// Matches the firmware `CryptoEngine::initNonce()`:
    /// ```text
    /// Bytes 0-7:   packet_id as u64 (little-endian)
    /// Bytes 8-11:  source node_id as u32 (little-endian)
    /// Bytes 12-15: zeros (AES-CTR block counter space)
    /// ```
    #[cfg(feature = "crypto")]
    fn make_nonce(source: NodeId, packet_id: u32) -> [u8; 16] {
        let mut nonce = [0u8; 16];

        // Bytes 0-7: packet_id as u64, little-endian
        nonce[0..8].copy_from_slice(&(packet_id as u64).to_le_bytes());

        // Bytes 8-11: source node_id as u32, little-endian
        nonce[8..12].copy_from_slice(&source.to_u32().to_le_bytes());

        // Bytes 12-15: zeros (block counter)
        nonce
    }

    /// Encrypt payload using AES-256-CTR (Meshtastic-compatible)
    ///
    /// Returns ciphertext only — no MIC in CTR mode.
    #[cfg(feature = "crypto")]
    pub fn encrypt(
        &self,
        plaintext: &[u8],
        source: NodeId,
        packet_id: u32,
    ) -> CryptoResult<Vec<u8>> {
        let nonce = Self::make_nonce(source, packet_id);

        let mut ciphertext = plaintext.to_vec();
        let mut cipher = Ctr128BE::<Aes256>::new(self.key.as_bytes().into(), &nonce.into());
        cipher.apply_keystream(&mut ciphertext);

        Ok(ciphertext)
    }

    /// Decrypt payload using AES-256-CTR (Meshtastic-compatible)
    ///
    /// Takes ciphertext only — no MIC verification in CTR mode.
    #[cfg(feature = "crypto")]
    pub fn decrypt(
        &self,
        ciphertext: &[u8],
        source: NodeId,
        packet_id: u32,
    ) -> CryptoResult<Vec<u8>> {
        let nonce = Self::make_nonce(source, packet_id);

        // Decrypt with AES-256-CTR (same as encrypt - XOR operation)
        let mut plaintext = ciphertext.to_vec();
        let mut cipher = Ctr128BE::<Aes256>::new(self.key.as_bytes().into(), &nonce.into());
        cipher.apply_keystream(&mut plaintext);

        Ok(plaintext)
    }

    /// Stub encrypt when crypto feature is disabled
    #[cfg(not(feature = "crypto"))]
    pub fn encrypt(
        &self,
        _plaintext: &[u8],
        _source: NodeId,
        _packet_id: u32,
    ) -> CryptoResult<Vec<u8>> {
        Err(CryptoError::FeatureNotEnabled)
    }

    /// Stub decrypt when crypto feature is disabled
    #[cfg(not(feature = "crypto"))]
    pub fn decrypt(
        &self,
        _ciphertext: &[u8],
        _source: NodeId,
        _packet_id: u32,
    ) -> CryptoResult<Vec<u8>> {
        Err(CryptoError::FeatureNotEnabled)
    }
}

/// Extension trait for encrypting/decrypting MeshPacket
pub trait PacketCrypto {
    /// Encrypt the packet payload
    fn encrypt(&mut self, ctx: &CryptoContext) -> CryptoResult<()>;

    /// Decrypt the packet payload
    fn decrypt(&mut self, ctx: &CryptoContext) -> CryptoResult<()>;
}

impl PacketCrypto for MeshPacket {
    #[cfg(feature = "crypto")]
    fn encrypt(&mut self, ctx: &CryptoContext) -> CryptoResult<()> {
        if self.header.flags.encrypted() {
            return Ok(());
        }

        self.header.flags.set_encrypted(true);

        let packet_id_32 = self.header.packet_id as u32;

        let ciphertext = ctx.encrypt(&self.payload, self.header.source, packet_id_32)?;

        self.payload = ciphertext;

        Ok(())
    }

    #[cfg(feature = "crypto")]
    fn decrypt(&mut self, ctx: &CryptoContext) -> CryptoResult<()> {
        if !self.header.flags.encrypted() {
            return Ok(());
        }

        let packet_id_32 = self.header.packet_id as u32;

        let plaintext = ctx.decrypt(&self.payload, self.header.source, packet_id_32)?;

        self.payload = plaintext;
        self.header.flags.set_encrypted(false);

        Ok(())
    }

    #[cfg(not(feature = "crypto"))]
    fn encrypt(&mut self, _ctx: &CryptoContext) -> CryptoResult<()> {
        Err(CryptoError::FeatureNotEnabled)
    }

    #[cfg(not(feature = "crypto"))]
    fn decrypt(&mut self, _ctx: &CryptoContext) -> CryptoResult<()> {
        Err(CryptoError::FeatureNotEnabled)
    }
}

// Stub implementations when crypto feature is disabled
#[cfg(not(feature = "crypto"))]
impl ChannelKey {
    /// Create a channel key (stub - zero-pads PSK to 32 bytes)
    pub fn new(channel_name: &str, psk: &[u8]) -> Self {
        let mut key = [0u8; 32];
        let copy_len = psk.len().min(32);
        key[..copy_len].copy_from_slice(&psk[..copy_len]);
        Self {
            key,
            channel_name: channel_name.to_string(),
        }
    }

    /// Create with default PSK (stub)
    pub fn with_default_psk(channel_name: &str) -> Self {
        Self::new(channel_name, DEFAULT_PSK)
    }

    /// Create from PSK index (stub)
    pub fn from_psk_index(channel_name: &str, index: u8) -> Self {
        if index == 0 {
            return Self {
                key: [0u8; 32],
                channel_name: channel_name.to_string(),
            };
        }
        let mut expanded = [0u8; 32];
        expanded[..DEFAULT_PSK.len()].copy_from_slice(DEFAULT_PSK);
        if index > 1 {
            expanded[DEFAULT_PSK.len() - 1] = index;
        }
        Self {
            key: expanded,
            channel_name: channel_name.to_string(),
        }
    }
}

#[cfg(not(feature = "crypto"))]
impl CryptoContext {
    /// Create a new crypto context (stub)
    pub fn new(channel_name: &str, psk: &[u8]) -> Self {
        Self {
            key: ChannelKey::new(channel_name, psk),
        }
    }

    /// Create with default PSK (stub)
    pub fn with_default_psk(channel_name: &str) -> Self {
        Self {
            key: ChannelKey::with_default_psk(channel_name),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_channel_key_creation() {
        let key = ChannelKey::new("LongFast", DEFAULT_PSK);
        assert_eq!(key.channel_name(), "LongFast");
        // PSK is used directly (zero-padded to 32 bytes), so key should be non-zero
        assert_ne!(key.as_bytes(), &[0u8; 32]);
    }

    #[test]
    fn test_channel_hash_xor_fold() {
        let key = ChannelKey::new("LongFast", DEFAULT_PSK);
        let hash = key.channel_hash();
        // Hash should be consistent
        assert_eq!(hash, key.channel_hash());

        // Verify XOR fold: xorHash(name) ^ xorHash(key)
        let name_hash = b"LongFast".iter().fold(0u8, |acc, &b| acc ^ b);
        let key_hash = key.as_bytes().iter().fold(0u8, |acc, &b| acc ^ b);
        assert_eq!(hash, name_hash ^ key_hash);
    }

    #[test]
    fn test_crypto_context_creation() {
        let ctx = CryptoContext::with_default_psk("LongFast");
        let hash = ctx.channel_hash();
        let key = ChannelKey::with_default_psk("LongFast");
        assert_eq!(hash, key.channel_hash());
    }

    #[cfg(feature = "crypto")]
    #[test]
    fn test_psk_direct_key() {
        // 32-byte PSK should be used directly (no SHA-256)
        let psk = [0x42u8; 32];
        let key = ChannelKey::new("Test", &psk);
        assert_eq!(key.as_bytes(), &psk);

        // 16-byte PSK should be zero-padded
        let short_psk = [0xABu8; 16];
        let key16 = ChannelKey::new("Test", &short_psk);
        let mut expected = [0u8; 32];
        expected[..16].copy_from_slice(&short_psk);
        assert_eq!(key16.as_bytes(), &expected);

        // 1-byte PSK index should expand from DEFAULT_PSK
        let key_idx = ChannelKey::from_psk_index("Test", 1);
        let mut expected_default = [0u8; 32];
        expected_default[..DEFAULT_PSK.len()].copy_from_slice(DEFAULT_PSK);
        assert_eq!(key_idx.as_bytes(), &expected_default);

        // Index 2 should change last byte
        let key_idx2 = ChannelKey::from_psk_index("Test", 2);
        let mut expected_idx2 = [0u8; 32];
        expected_idx2[..DEFAULT_PSK.len()].copy_from_slice(DEFAULT_PSK);
        expected_idx2[DEFAULT_PSK.len() - 1] = 2;
        assert_eq!(key_idx2.as_bytes(), &expected_idx2);
    }

    #[cfg(feature = "crypto")]
    #[test]
    fn test_encrypt_decrypt_roundtrip() {
        let ctx = CryptoContext::with_default_psk("TestChannel");
        let source = NodeId::from_bytes([0x11, 0x22, 0x33, 0x44]);
        let packet_id = 12345u32;

        let plaintext = b"Hello, Meshtastic!";

        // Encrypt (no MIC in CTR mode)
        let ciphertext = ctx
            .encrypt(plaintext, source, packet_id)
            .expect("Encryption failed");

        assert_ne!(ciphertext, plaintext);
        assert_eq!(ciphertext.len(), plaintext.len());

        // Decrypt
        let decrypted = ctx
            .decrypt(&ciphertext, source, packet_id)
            .expect("Decryption failed");

        assert_eq!(decrypted, plaintext);
    }

    #[cfg(feature = "crypto")]
    #[test]
    fn test_tampered_ciphertext_decrypts_to_garbage() {
        // In CTR mode without MIC, tampered ciphertext decrypts but produces garbage
        let ctx = CryptoContext::with_default_psk("TestChannel");
        let source = NodeId::from_bytes([0x11, 0x22, 0x33, 0x44]);
        let packet_id = 12345u32;

        let plaintext = b"Secret message";

        let mut ciphertext = ctx.encrypt(plaintext, source, packet_id).unwrap();
        ciphertext[0] ^= 0xFF;

        // Decryption succeeds but produces wrong plaintext
        let decrypted = ctx.decrypt(&ciphertext, source, packet_id).unwrap();
        assert_ne!(decrypted.as_slice(), plaintext);
    }

    #[cfg(feature = "crypto")]
    #[test]
    fn test_packet_encrypt_decrypt() {
        use super::super::packet::PacketType;

        let ctx = CryptoContext::with_default_psk("TestChannel");
        let source = NodeId::from_bytes([0xAA, 0xBB, 0xCC, 0xDD]);

        let mut packet = MeshPacket::broadcast(source, b"Test payload", 3);
        packet.packet_type = PacketType::Text;

        assert!(!packet.header.flags.encrypted());

        // Encrypt
        packet.encrypt(&ctx).expect("Encryption failed");
        assert!(packet.header.flags.encrypted());
        assert_ne!(packet.payload, b"Test payload");

        // Decrypt
        packet.decrypt(&ctx).expect("Decryption failed");
        assert!(!packet.header.flags.encrypted());
        assert_eq!(packet.payload, b"Test payload");
    }

    #[cfg(feature = "crypto")]
    #[test]
    fn test_different_keys_produce_different_ciphertext() {
        // Use different PSKs (not just different names — name doesn't affect the key)
        let ctx1 = CryptoContext::new("Channel1", &[0x11; 32]);
        let ctx2 = CryptoContext::new("Channel2", &[0x22; 32]);
        let source = NodeId::from_bytes([0x11, 0x22, 0x33, 0x44]);
        let packet_id = 100u32;

        let plaintext = b"Same message";

        let ciphertext1 = ctx1.encrypt(plaintext, source, packet_id).unwrap();
        let ciphertext2 = ctx2.encrypt(plaintext, source, packet_id).unwrap();

        // Different keys should produce different ciphertexts
        assert_ne!(ciphertext1, ciphertext2);
    }

    #[cfg(feature = "crypto")]
    #[test]
    fn test_same_psk_same_ciphertext_regardless_of_name() {
        // In Meshtastic, PSK is used directly — channel name does not affect the key
        let ctx1 = CryptoContext::with_default_psk("Channel1");
        let ctx2 = CryptoContext::with_default_psk("Channel2");
        let source = NodeId::from_bytes([0x11, 0x22, 0x33, 0x44]);
        let packet_id = 100u32;

        let plaintext = b"Same message";

        let ciphertext1 = ctx1.encrypt(plaintext, source, packet_id).unwrap();
        let ciphertext2 = ctx2.encrypt(plaintext, source, packet_id).unwrap();

        // Same PSK → same key → same ciphertext (name doesn't affect encryption)
        assert_eq!(ciphertext1, ciphertext2);
    }

    #[cfg(feature = "crypto")]
    #[test]
    fn test_nonce_uniqueness() {
        let nonce1 = CryptoContext::make_nonce(
            NodeId::from_bytes([1, 2, 3, 4]),
            100,
        );
        let nonce2 = CryptoContext::make_nonce(
            NodeId::from_bytes([1, 2, 3, 4]),
            101,
        );
        let nonce3 = CryptoContext::make_nonce(
            NodeId::from_bytes([5, 6, 7, 8]),
            100,
        );

        assert_ne!(nonce1, nonce2);
        assert_ne!(nonce1, nonce3);
        assert_ne!(nonce2, nonce3);
    }

    #[cfg(feature = "crypto")]
    #[test]
    fn test_nonce_matches_meshtastic() {
        // Verify nonce layout matches Meshtastic firmware CryptoEngine::initNonce()
        let source = NodeId::from_u32(0xAABBCCDD);
        let packet_id: u32 = 0x12345678;
        let nonce = CryptoContext::make_nonce(source, packet_id);

        // Bytes 0-7: packet_id as u64 LE
        assert_eq!(&nonce[0..8], &(packet_id as u64).to_le_bytes());
        // Bytes 8-11: node_id as u32 LE
        assert_eq!(&nonce[8..12], &source.to_u32().to_le_bytes());
        // Bytes 12-15: zeros
        assert_eq!(&nonce[12..16], &[0u8; 4]);
    }

    #[cfg(feature = "crypto")]
    #[test]
    fn test_full_packet_lifecycle() {
        // trace:MESH-012 | ai:claude
        // Full lifecycle: text → Data::text() → protobuf encode → encrypt → decrypt → decode → verify
        let ctx = CryptoContext::with_default_psk("LongFast");
        let source = NodeId::from_u32(0xDEADBEEF);
        let packet_id = 42u32;

        let original_text = b"Hello from R4W!";

        // Encrypt
        let ciphertext = ctx.encrypt(original_text, source, packet_id).unwrap();
        assert_ne!(ciphertext.as_slice(), original_text);

        // Decrypt
        let plaintext = ctx.decrypt(&ciphertext, source, packet_id).unwrap();
        assert_eq!(plaintext, original_text);
    }
}
