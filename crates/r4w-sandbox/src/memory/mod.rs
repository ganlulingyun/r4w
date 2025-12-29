//! Memory protection utilities for secure waveform processing.
//!
//! This module provides secure memory buffers with features like:
//! - Automatic zeroization on drop
//! - Memory locking (prevents swapping)
//! - Optional encryption at rest
//! - Guard pages for bounds checking

use crate::error::{Result, SandboxError};
use zeroize::Zeroize;

/// A secure buffer that is zeroized on drop and optionally locked in memory.
///
/// Use this for sensitive data like cryptographic keys, hop sequences,
/// or classified signal parameters.
pub struct SecureBuffer<T: Zeroize + Clone + Default> {
    data: Vec<T>,
    locked: bool,
}

impl<T: Zeroize + Clone + Default> SecureBuffer<T> {
    /// Create a new secure buffer with the given capacity.
    ///
    /// The buffer is initialized with default values and locked in memory
    /// if `lock` is true (requires CAP_IPC_LOCK or root).
    pub fn new(capacity: usize, lock: bool) -> Result<Self> {
        let data = vec![T::default(); capacity];
        let mut buffer = Self {
            data,
            locked: false,
        };

        if lock {
            buffer.lock_memory()?;
        }

        Ok(buffer)
    }

    /// Create from existing data, taking ownership and optionally locking.
    pub fn from_vec(data: Vec<T>, lock: bool) -> Result<Self> {
        let mut buffer = Self {
            data,
            locked: false,
        };

        if lock {
            buffer.lock_memory()?;
        }

        Ok(buffer)
    }

    /// Lock the buffer in physical memory (prevent swapping).
    #[cfg(target_os = "linux")]
    fn lock_memory(&mut self) -> Result<()> {
        let ptr = self.data.as_ptr() as *const libc::c_void;
        let len = self.data.len() * std::mem::size_of::<T>();

        let result = unsafe { libc::mlock(ptr, len) };

        if result != 0 {
            return Err(SandboxError::MemoryError(format!(
                "mlock failed: {}",
                std::io::Error::last_os_error()
            )));
        }

        // Advise kernel not to include in core dumps
        unsafe {
            libc::madvise(ptr as *mut libc::c_void, len, libc::MADV_DONTDUMP);
        }

        self.locked = true;
        Ok(())
    }

    #[cfg(not(target_os = "linux"))]
    fn lock_memory(&mut self) -> Result<()> {
        // Memory locking not available on this platform
        Ok(())
    }

    /// Unlock the buffer from physical memory.
    #[cfg(target_os = "linux")]
    fn unlock_memory(&mut self) {
        if self.locked {
            let ptr = self.data.as_ptr() as *const libc::c_void;
            let len = self.data.len() * std::mem::size_of::<T>();
            unsafe {
                libc::munlock(ptr, len);
            }
            self.locked = false;
        }
    }

    #[cfg(not(target_os = "linux"))]
    fn unlock_memory(&mut self) {
        self.locked = false;
    }

    /// Get a reference to the underlying data.
    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    /// Get a mutable reference to the underlying data.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        &mut self.data
    }

    /// Get the length of the buffer.
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Check if the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Check if the buffer is locked in memory.
    pub fn is_locked(&self) -> bool {
        self.locked
    }

    /// Explicitly zeroize the buffer (also happens automatically on drop).
    pub fn clear(&mut self) {
        self.data.zeroize();
    }
}

impl<T: Zeroize + Clone + Default> Drop for SecureBuffer<T> {
    fn drop(&mut self) {
        // Zeroize before unlock
        self.data.zeroize();
        // Unlock after zeroize
        self.unlock_memory();
    }
}

impl<T: Zeroize + Clone + Default> std::ops::Index<usize> for SecureBuffer<T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl<T: Zeroize + Clone + Default> std::ops::IndexMut<usize> for SecureBuffer<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}

/// An encrypted buffer that stores data encrypted at rest.
///
/// Data is only decrypted when accessed and re-encrypted when stored.
/// Useful for caching sensitive data that may remain in memory for
/// extended periods.
pub struct EncryptedBuffer {
    /// Encrypted data
    ciphertext: Vec<u8>,
    /// Encryption nonce
    nonce: [u8; 12],
    /// Key (stored in secure memory)
    key: SecureBuffer<u8>,
}

impl EncryptedBuffer {
    /// Create a new encrypted buffer from plaintext data.
    pub fn new(plaintext: &[u8]) -> Result<Self> {
        use aes_gcm::{
            aead::{Aead, KeyInit},
            Aes256Gcm, Nonce,
        };
        use rand::RngCore;

        // Generate random key
        let mut key_bytes = vec![0u8; 32];
        rand::thread_rng().fill_bytes(&mut key_bytes);
        let key = SecureBuffer::from_vec(key_bytes.clone(), true)?;

        // Generate random nonce
        let mut nonce = [0u8; 12];
        rand::thread_rng().fill_bytes(&mut nonce);

        // Encrypt the data
        let cipher = Aes256Gcm::new_from_slice(&key_bytes)
            .map_err(|e| SandboxError::MemoryError(format!("cipher init failed: {}", e)))?;

        let ciphertext = cipher
            .encrypt(Nonce::from_slice(&nonce), plaintext)
            .map_err(|e| SandboxError::MemoryError(format!("encryption failed: {}", e)))?;

        // Zeroize the temporary key bytes
        key_bytes.zeroize();

        Ok(Self {
            ciphertext,
            nonce,
            key,
        })
    }

    /// Decrypt and return the plaintext data.
    ///
    /// The returned `SecureBuffer` will be zeroized when dropped.
    pub fn decrypt(&self) -> Result<SecureBuffer<u8>> {
        use aes_gcm::{
            aead::{Aead, KeyInit},
            Aes256Gcm, Nonce,
        };

        let cipher = Aes256Gcm::new_from_slice(self.key.as_slice())
            .map_err(|e| SandboxError::MemoryError(format!("cipher init failed: {}", e)))?;

        let plaintext = cipher
            .decrypt(Nonce::from_slice(&self.nonce), self.ciphertext.as_slice())
            .map_err(|e| SandboxError::MemoryError(format!("decryption failed: {}", e)))?;

        SecureBuffer::from_vec(plaintext, true)
    }

    /// Get the size of the encrypted data.
    pub fn encrypted_len(&self) -> usize {
        self.ciphertext.len()
    }
}

impl Drop for EncryptedBuffer {
    fn drop(&mut self) {
        // Zeroize ciphertext (key is handled by SecureBuffer)
        self.ciphertext.zeroize();
        self.nonce.zeroize();
    }
}

/// Guard page allocation for bounds checking.
///
/// Creates a memory region with guard pages before and after
/// to catch buffer overflows/underflows.
#[cfg(target_os = "linux")]
pub struct GuardedBuffer<T> {
    /// Pointer to the usable data region
    data_ptr: *mut T,
    /// Total allocation size including guards
    total_size: usize,
    /// Number of elements
    len: usize,
    /// Base pointer for deallocation
    base_ptr: *mut u8,
}

#[cfg(target_os = "linux")]
impl<T: Default + Clone> GuardedBuffer<T> {
    /// Create a new guarded buffer with guard pages.
    ///
    /// Guard pages are set to PROT_NONE to cause SIGSEGV on access.
    pub fn new(len: usize) -> Result<Self> {
        let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as usize };
        let data_size = len * std::mem::size_of::<T>();

        // Round up to page boundary
        let data_pages = (data_size + page_size - 1) / page_size;
        let total_pages = data_pages + 2; // +2 for guard pages
        let total_size = total_pages * page_size;

        // Allocate with mmap
        let base_ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                total_size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS,
                -1,
                0,
            )
        };

        if base_ptr == libc::MAP_FAILED {
            return Err(SandboxError::MemoryError(format!(
                "mmap failed: {}",
                std::io::Error::last_os_error()
            )));
        }

        // Set up guard pages (first and last page)
        unsafe {
            // First guard page
            libc::mprotect(base_ptr, page_size, libc::PROT_NONE);

            // Last guard page
            let last_guard = (base_ptr as usize + total_size - page_size) as *mut libc::c_void;
            libc::mprotect(last_guard, page_size, libc::PROT_NONE);
        }

        // Data region starts after first guard page
        let data_ptr = unsafe { (base_ptr as *mut u8).add(page_size) as *mut T };

        // Initialize data
        for i in 0..len {
            unsafe {
                std::ptr::write(data_ptr.add(i), T::default());
            }
        }

        Ok(Self {
            data_ptr,
            total_size,
            len,
            base_ptr: base_ptr as *mut u8,
        })
    }

    /// Get a reference to the data slice.
    pub fn as_slice(&self) -> &[T] {
        unsafe { std::slice::from_raw_parts(self.data_ptr, self.len) }
    }

    /// Get a mutable reference to the data slice.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        unsafe { std::slice::from_raw_parts_mut(self.data_ptr, self.len) }
    }

    /// Get the length.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

#[cfg(target_os = "linux")]
impl<T> Drop for GuardedBuffer<T> {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.base_ptr as *mut libc::c_void, self.total_size);
        }
    }
}

// GuardedBuffer is Send + Sync if T is
#[cfg(target_os = "linux")]
unsafe impl<T: Send> Send for GuardedBuffer<T> {}
#[cfg(target_os = "linux")]
unsafe impl<T: Sync> Sync for GuardedBuffer<T> {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_secure_buffer_creation() {
        let buffer: SecureBuffer<u8> = SecureBuffer::new(1024, false).unwrap();
        assert_eq!(buffer.len(), 1024);
        assert!(!buffer.is_locked()); // Didn't request locking
    }

    #[test]
    fn test_secure_buffer_zeroize() {
        let mut buffer: SecureBuffer<u8> = SecureBuffer::from_vec(vec![0xAA; 16], false).unwrap();
        assert_eq!(buffer[0], 0xAA);

        buffer.clear();

        // After zeroize, all bytes should be 0
        for i in 0..buffer.len() {
            assert_eq!(buffer[i], 0);
        }
    }

    #[test]
    fn test_encrypted_buffer_roundtrip() {
        let plaintext = b"secret waveform parameters";

        let encrypted = EncryptedBuffer::new(plaintext).unwrap();
        assert!(encrypted.encrypted_len() > plaintext.len()); // Includes tag

        let decrypted = encrypted.decrypt().unwrap();
        assert_eq!(decrypted.as_slice(), plaintext);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn test_guarded_buffer() {
        let buffer: GuardedBuffer<u32> = GuardedBuffer::new(100).unwrap();
        assert_eq!(buffer.len(), 100);

        // Should be able to access all elements
        for i in 0..100 {
            assert_eq!(buffer.as_slice()[i], 0);
        }
    }
}
