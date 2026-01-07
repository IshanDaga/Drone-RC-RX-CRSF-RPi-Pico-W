#ifndef SECURITY_H
#define SECURITY_H

#include <Arduino.h>
#include <stdint.h>
#if defined(ARDUINO_ARCH_RP2040)
#include <hardware/flash.h>
#endif

#define PAIRING_KEY_SIZE 16
#define HMAC_SIZE 4  // Truncated HMAC-SHA256 (first 4 bytes)
#define SEQUENCE_SIZE 2
#define DEVICE_ID_SIZE 8  // 8-byte unique device identifier
#define BINDING_UID_SIZE 8  // 8-byte UID derived from binding phrase

// EEPROM magic to invalidate old stored state when changed
#define SECURITY_MAGIC_VALUE 0x54554352UL  // "TECR"
#define SECURITY_MAGIC_ADDR 120            // 4 bytes

// Compile-time default binding phrase (can be overridden)
#ifndef DEFAULT_BINDING_PHRASE
#define DEFAULT_BINDING_PHRASE "FPV-DEFAULT-2024"
#endif

// EEPROM addresses
#define PAIRING_KEY_ADDR 128
#define DEVICE_ID_ADDR 144  // After pairing key (128 + 16)
#define PAIRED_DEVICE_ID_ADDR 152  // After device ID (144 + 8)
#define BINDING_UID_ADDR 160  // After paired device ID (152 + 8)

class Security {
public:
    Security();
    
    // Initialize security (load pairing key from EEPROM or generate new one)
    bool begin();
    
    // Generate or set pairing key
    bool generatePairingKey();
    bool setPairingKey(const uint8_t* key);
    bool getPairingKey(uint8_t* key);
    
    // Calculate HMAC for data (truncated to 4 bytes)
    void calculateHMAC(const uint8_t* data, size_t dataLen, uint8_t* hmac);
    
    // Verify HMAC
    bool verifyHMAC(const uint8_t* data, size_t dataLen, const uint8_t* hmac);
    
    // Check if pairing key exists
    bool hasPairingKey();
    
    // Device ID management
    bool generateDeviceId();
    bool getDeviceId(uint8_t* deviceId);
    bool setPairedDeviceId(const uint8_t* deviceId);
    bool getPairedDeviceId(uint8_t* deviceId);
    bool hasPairedDeviceId();
    
    // Encryption/Decryption (AES-128)
    bool encrypt(const uint8_t* plaintext, size_t len, uint8_t* ciphertext, uint16_t sequence, const uint8_t* deviceId);
    bool decrypt(const uint8_t* ciphertext, size_t len, uint8_t* plaintext, uint16_t sequence, const uint8_t* deviceId);
    
    // Binding phrase UID management
    bool generateBindingUID(const char* bindingPhrase);
    bool getBindingUID(uint8_t* uid);
    bool hasBindingUID();
    bool verifyBindingUID(const uint8_t* uid);

    // Load device ID from silicon (preferred on RP2040)
    bool loadDeviceIdFromSilicon();
    
private:
    uint8_t _pairingKey[PAIRING_KEY_SIZE];
    bool _keyLoaded;
    uint8_t _deviceId[DEVICE_ID_SIZE];
    bool _deviceIdLoaded;
    uint8_t _pairedDeviceId[DEVICE_ID_SIZE];
    bool _pairedDeviceIdLoaded;
    uint8_t _bindingUID[BINDING_UID_SIZE];
    bool _bindingUIDLoaded;
    
    // Simple HMAC-SHA256 implementation (lightweight)
    void sha256(const uint8_t* data, size_t len, uint8_t* hash);
    void hmac_sha256(const uint8_t* key, size_t keyLen, const uint8_t* data, size_t dataLen, uint8_t* hmac);
};

#endif // SECURITY_H

