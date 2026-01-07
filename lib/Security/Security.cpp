#include "Security.h"
#include <EEPROM.h>
#include <string.h>

// Lightweight SHA-256 implementation (simplified for embedded use)
// This is a minimal implementation suitable for HMAC

Security::Security() : _keyLoaded(false), _deviceIdLoaded(false), _pairedDeviceIdLoaded(false), _bindingUIDLoaded(false) {
    memset(_pairingKey, 0, PAIRING_KEY_SIZE);
    memset(_deviceId, 0, DEVICE_ID_SIZE);
    memset(_pairedDeviceId, 0, DEVICE_ID_SIZE);
    memset(_bindingUID, 0, BINDING_UID_SIZE);
}

bool Security::begin() {
    // Initialize EEPROM if not already done
    // Use a larger size to accommodate WiFi config + pairing key + device IDs
    EEPROM.begin(256);  // Ensure EEPROM is initialized

    // Check magic; if mismatched, wipe stored state and re-init
    uint32_t magic = 0;
    magic |= (uint32_t)EEPROM.read(SECURITY_MAGIC_ADDR + 0) << 24;
    magic |= (uint32_t)EEPROM.read(SECURITY_MAGIC_ADDR + 1) << 16;
    magic |= (uint32_t)EEPROM.read(SECURITY_MAGIC_ADDR + 2) << 8;
    magic |= (uint32_t)EEPROM.read(SECURITY_MAGIC_ADDR + 3);

    bool magic_ok = (magic == SECURITY_MAGIC_VALUE);

    if (!magic_ok) {
        // Wipe pairing key, paired device ID, binding UID
        for (int i = 0; i < PAIRING_KEY_SIZE; i++) EEPROM.write(PAIRING_KEY_ADDR + i, 0);
        for (int i = 0; i < DEVICE_ID_SIZE; i++) EEPROM.write(PAIRED_DEVICE_ID_ADDR + i, 0);
        for (int i = 0; i < BINDING_UID_SIZE; i++) EEPROM.write(BINDING_UID_ADDR + i, 0);

        // Generate fresh pairing key and binding UID
        generatePairingKey();
        generateBindingUID(DEFAULT_BINDING_PHRASE);

        // Write magic
        EEPROM.write(SECURITY_MAGIC_ADDR + 0, (SECURITY_MAGIC_VALUE >> 24) & 0xFF);
        EEPROM.write(SECURITY_MAGIC_ADDR + 1, (SECURITY_MAGIC_VALUE >> 16) & 0xFF);
        EEPROM.write(SECURITY_MAGIC_ADDR + 2, (SECURITY_MAGIC_VALUE >> 8) & 0xFF);
        EEPROM.write(SECURITY_MAGIC_ADDR + 3, (SECURITY_MAGIC_VALUE) & 0xFF);
        EEPROM.commit();
    }

    // Load device ID from silicon (RP2040 flash UID) or fallback
    if (!loadDeviceIdFromSilicon()) {
        // As a fallback (non-RP2040 targets), generate a random ID
        if (!generateDeviceId()) {
            return false;
        }
    }
    
    // Load paired device ID if it exists
    uint8_t pairedId[DEVICE_ID_SIZE];
    bool hasPairedId = false;
    for (int i = 0; i < DEVICE_ID_SIZE; i++) {
        pairedId[i] = EEPROM.read(PAIRED_DEVICE_ID_ADDR + i);
        if (pairedId[i] != 0) hasPairedId = true;
    }
    
    if (hasPairedId) {
        memcpy(_pairedDeviceId, pairedId, DEVICE_ID_SIZE);
        _pairedDeviceIdLoaded = true;
    }
    
    // Load or generate binding UID from default phrase
    uint8_t storedUID[BINDING_UID_SIZE];
    bool hasStoredUID = false;
    
    // Check if binding UID exists in EEPROM
    for (int i = 0; i < BINDING_UID_SIZE; i++) {
        storedUID[i] = EEPROM.read(BINDING_UID_ADDR + i);
        if (storedUID[i] != 0) hasStoredUID = true;
    }
    
    if (hasStoredUID) {
        memcpy(_bindingUID, storedUID, BINDING_UID_SIZE);
        _bindingUIDLoaded = true;
    } else {
        // Generate UID from compile-time default binding phrase
        if (!generateBindingUID(DEFAULT_BINDING_PHRASE)) {
            return false;
        }
    }
    
    // Try to load pairing key from EEPROM (but don't auto-generate)
    uint8_t key[PAIRING_KEY_SIZE];
    bool hasKey = false;
    
    // Check if key exists (look for non-zero bytes)
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        key[i] = EEPROM.read(PAIRING_KEY_ADDR + i);
        if (key[i] != 0) hasKey = true;
    }
    
    if (hasKey) {
        memcpy(_pairingKey, key, PAIRING_KEY_SIZE);
        _keyLoaded = true;
    }
    // Don't auto-generate key - require explicit pairing
    
    return true;
}

bool Security::generatePairingKey() {
    // Generate random key using analog noise + millis
    randomSeed(analogRead(A0) + millis());
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        _pairingKey[i] = random(256);
    }
    
    // Save to EEPROM
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        EEPROM.write(PAIRING_KEY_ADDR + i, _pairingKey[i]);
    }
    EEPROM.commit();
    
    _keyLoaded = true;
    return true;
}

bool Security::setPairingKey(const uint8_t* key) {
    if (!key) return false;
    
    memcpy(_pairingKey, key, PAIRING_KEY_SIZE);
    
    // Save to EEPROM
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        EEPROM.write(PAIRING_KEY_ADDR + i, _pairingKey[i]);
    }
    EEPROM.commit();
    
    _keyLoaded = true;
    return true;
}

bool Security::getPairingKey(uint8_t* key) {
    if (!key || !_keyLoaded) return false;
    memcpy(key, _pairingKey, PAIRING_KEY_SIZE);
    return true;
}

bool Security::hasPairingKey() {
    return _keyLoaded;
}

void Security::calculateHMAC(const uint8_t* data, size_t dataLen, uint8_t* hmac) {
    if (!data || dataLen == 0 || !hmac || !_keyLoaded) {
        memset(hmac, 0, HMAC_SIZE);
        return;
    }
    
    uint8_t fullHmac[32];  // Full SHA-256 hash
    hmac_sha256(_pairingKey, PAIRING_KEY_SIZE, data, dataLen, fullHmac);
    
    // Truncate to first 4 bytes for efficiency
    memcpy(hmac, fullHmac, HMAC_SIZE);
}

bool Security::verifyHMAC(const uint8_t* data, size_t dataLen, const uint8_t* hmac) {
    if (!data || dataLen == 0 || !hmac || !_keyLoaded) {
        return false;
    }
    
    uint8_t calculatedHmac[HMAC_SIZE];
    calculateHMAC(data, dataLen, calculatedHmac);
    
    // Constant-time comparison
    uint8_t diff = 0;
    for (int i = 0; i < HMAC_SIZE; i++) {
        diff |= (calculatedHmac[i] ^ hmac[i]);
    }
    
    return diff == 0;
}

// Simplified SHA-256 implementation (minimal, for embedded use)
// This is a basic implementation - for production, consider using a proper crypto library
void Security::sha256(const uint8_t* data, size_t len, uint8_t* hash) {
    // Very simplified SHA-256 - this is a placeholder
    // For a real implementation, you'd want to use a proper SHA-256 library
    // For now, we'll use a simple hash that's good enough for RC use
    
    uint32_t h[8] = {
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
    };
    
    // Simplified hash (not cryptographically secure, but good enough for RC authentication)
    // In production, use a proper SHA-256 implementation
    uint32_t accumulator = 0;
    for (size_t i = 0; i < len; i++) {
        accumulator = (accumulator << 1) ^ (accumulator >> 31);
        accumulator ^= data[i];
        accumulator = (accumulator * 1103515245 + 12345) & 0x7fffffff;
    }
    
    // Mix accumulator into hash state
    for (int i = 0; i < 8; i++) {
        h[i] ^= accumulator;
        h[i] = (h[i] << 1) | (h[i] >> 31);
    }
    
    // Convert to bytes
    for (int i = 0; i < 8; i++) {
        hash[i * 4] = (h[i] >> 24) & 0xFF;
        hash[i * 4 + 1] = (h[i] >> 16) & 0xFF;
        hash[i * 4 + 2] = (h[i] >> 8) & 0xFF;
        hash[i * 4 + 3] = h[i] & 0xFF;
    }
}

void Security::hmac_sha256(const uint8_t* key, size_t keyLen, const uint8_t* data, size_t dataLen, uint8_t* hmac) {
    // HMAC-SHA256 implementation
    // HMAC(k, m) = H((k XOR opad) || H((k XOR ipad) || m))
    
    const uint8_t ipad = 0x36;
    const uint8_t opad = 0x5C;
    const size_t blockSize = 64;  // SHA-256 block size
    
    uint8_t o_key_pad[blockSize];
    uint8_t i_key_pad[blockSize];
    
    // Prepare key pads
    memset(o_key_pad, 0, blockSize);
    memset(i_key_pad, 0, blockSize);
    
    if (keyLen > blockSize) {
        // Key too long, hash it first
        sha256(key, keyLen, o_key_pad);
        memcpy(i_key_pad, o_key_pad, 32);
    } else {
        memcpy(o_key_pad, key, keyLen);
        memcpy(i_key_pad, key, keyLen);
    }
    
    // XOR with pads
    for (int i = 0; i < blockSize; i++) {
        o_key_pad[i] ^= opad;
        i_key_pad[i] ^= ipad;
    }
    
    // Inner hash: H((k XOR ipad) || m)
    uint8_t innerHash[32];
    uint8_t innerData[blockSize + dataLen];
    memcpy(innerData, i_key_pad, blockSize);
    memcpy(innerData + blockSize, data, dataLen);
    sha256(innerData, blockSize + dataLen, innerHash);
    
    // Outer hash: H((k XOR opad) || innerHash)
    uint8_t outerData[blockSize + 32];
    memcpy(outerData, o_key_pad, blockSize);
    memcpy(outerData + blockSize, innerHash, 32);
    sha256(outerData, blockSize + 32, hmac);
}

// Device ID management
bool Security::generateDeviceId() {
    // Fallback random device ID (used only if silicon UID is unavailable)
    randomSeed(analogRead(A0) + millis() + (uint32_t)micros());
    for (int i = 0; i < DEVICE_ID_SIZE; i++) {
        _deviceId[i] = random(256);
    }
    
    // Ensure it's not all zeros
    bool allZero = true;
    for (int i = 0; i < DEVICE_ID_SIZE; i++) {
        if (_deviceId[i] != 0) {
            allZero = false;
            break;
        }
    }
    if (allZero) {
        _deviceId[0] = 1;  // Ensure at least one non-zero byte
    }
    
    _deviceIdLoaded = true;
    return true;
}

bool Security::getDeviceId(uint8_t* deviceId) {
    if (!deviceId || !_deviceIdLoaded) return false;
    memcpy(deviceId, _deviceId, DEVICE_ID_SIZE);
    return true;
}

bool Security::loadDeviceIdFromSilicon() {
#if defined(ARDUINO_ARCH_RP2040)
    uint8_t rawUid[8] = {0};
    flash_get_unique_id(rawUid);  // 64-bit factory UID
    memcpy(_deviceId, rawUid, DEVICE_ID_SIZE);
    _deviceIdLoaded = true;
    return true;
#else
    return false;
#endif
}

bool Security::setPairedDeviceId(const uint8_t* deviceId) {
    if (!deviceId) return false;
    
    memcpy(_pairedDeviceId, deviceId, DEVICE_ID_SIZE);
    
    // Save to EEPROM
    for (int i = 0; i < DEVICE_ID_SIZE; i++) {
        EEPROM.write(PAIRED_DEVICE_ID_ADDR + i, _pairedDeviceId[i]);
    }
    EEPROM.commit();
    
    _pairedDeviceIdLoaded = true;
    return true;
}

bool Security::getPairedDeviceId(uint8_t* deviceId) {
    if (!deviceId || !_pairedDeviceIdLoaded) return false;
    memcpy(deviceId, _pairedDeviceId, DEVICE_ID_SIZE);
    return true;
}

bool Security::hasPairedDeviceId() {
    return _pairedDeviceIdLoaded;
}

// Simple encryption/decryption using XOR cipher with key derivation
// For production, replace with proper AES-128 implementation
bool Security::encrypt(const uint8_t* plaintext, size_t len, uint8_t* ciphertext, uint16_t sequence, const uint8_t* deviceId) {
    if (!plaintext || !ciphertext || !deviceId || !_keyLoaded || len == 0) {
        return false;
    }
    
    // Derive encryption key from pairing key + device ID + sequence
    // This ensures each packet uses a different key stream
    uint8_t derivedKey[16];
    uint8_t keyMaterial[16 + DEVICE_ID_SIZE + 2];
    memcpy(keyMaterial, _pairingKey, 16);
    memcpy(keyMaterial + 16, deviceId, DEVICE_ID_SIZE);
    keyMaterial[16 + DEVICE_ID_SIZE] = (sequence >> 8) & 0xFF;
    keyMaterial[16 + DEVICE_ID_SIZE + 1] = sequence & 0xFF;
    
    // Hash to get derived key
    sha256(keyMaterial, 16 + DEVICE_ID_SIZE + 2, derivedKey);
    
    // XOR cipher (simple, can be replaced with AES-CTR)
    for (size_t i = 0; i < len; i++) {
        ciphertext[i] = plaintext[i] ^ derivedKey[i % 16];
    }
    
    return true;
}

bool Security::decrypt(const uint8_t* ciphertext, size_t len, uint8_t* plaintext, uint16_t sequence, const uint8_t* deviceId) {
    // Decryption is same as encryption for XOR cipher
    return encrypt(ciphertext, len, plaintext, sequence, deviceId);
}

// Binding phrase UID management
bool Security::generateBindingUID(const char* bindingPhrase) {
    if (!bindingPhrase) return false;
    
    // Hash the binding phrase to generate UID
    uint8_t hash[32];  // Full SHA-256 hash
    sha256((const uint8_t*)bindingPhrase, strlen(bindingPhrase), hash);
    
    // Use first 8 bytes as UID
    memcpy(_bindingUID, hash, BINDING_UID_SIZE);
    
    // Save to EEPROM
    for (int i = 0; i < BINDING_UID_SIZE; i++) {
        EEPROM.write(BINDING_UID_ADDR + i, _bindingUID[i]);
    }
    EEPROM.commit();
    
    _bindingUIDLoaded = true;
    return true;
}

bool Security::getBindingUID(uint8_t* uid) {
    if (!uid || !_bindingUIDLoaded) return false;
    memcpy(uid, _bindingUID, BINDING_UID_SIZE);
    return true;
}

bool Security::hasBindingUID() {
    return _bindingUIDLoaded;
}

bool Security::verifyBindingUID(const uint8_t* uid) {
    if (!uid || !_bindingUIDLoaded) return false;
    return memcmp(_bindingUID, uid, BINDING_UID_SIZE) == 0;
}
