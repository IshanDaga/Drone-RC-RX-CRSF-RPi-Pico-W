#include "pairing.h"
#include "Security.h"
#include "SX128xLink.h"
#include <string.h>

// RP2040-specific includes for BOOTSEL button
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

// Helper must run from RAM because we temporarily disable the XIP flash
// interface by driving QSPI CS; executing from flash while CS is forced
// low will hard fault/lock up.
static bool __not_in_flash_func(readBootselButton)() {
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Brief delay for pin to settle
    for (volatile int i = 0; i < 1000; ++i)
        ;

    // Read the pin state (low = pressed)
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

    // Restore chip select to normal operation
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);

    return button_state;
}

// ============================================================
// TX Side: Send pairing key
// ============================================================
void Pairing::sendPairingKey(SX128xLink* radio, Security* security, 
                              const uint8_t* txDeviceId, bool* pairingMode, 
                              unsigned long* lastPairingSend) {
    if (!radio || !security || !txDeviceId || !pairingMode || !lastPairingSend) {
        return;
    }
    
    if (!radio->isReady() || !*pairingMode) {
        return;
    }
    
    // Rate limit pairing packets
    if (millis() - *lastPairingSend < PAIRING_SCAN_INTERVAL_MS) {
        return;
    }
    *lastPairingSend = millis();
    
    // Must have binding UID to pair
    if (!security->hasBindingUID()) {
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 5000) {
            Serial.println("[TX] ERROR: No binding UID - cannot pair!");
            lastLog = millis();
        }
        return;
    }
    
    // Get binding UID
    uint8_t bindingUID[BINDING_UID_SIZE];
    if (!security->getBindingUID(bindingUID)) {
        return;
    }
    
    // Generate pairing key if we don't have one
    if (!security->hasPairingKey()) {
        security->generatePairingKey();
    }
    
    // Get pairing key
    uint8_t key[PAIRING_KEY_SIZE];
    if (!security->getPairingKey(key)) {
        return;
    }
    
    // Pairing packet format: [MSG_TYPE: 1][BINDING_UID: 8][TX_DEVICE_ID: 8][PAIRING_KEY: 16] = 33 bytes
    // Binding UID must match for RX to accept pairing
    uint8_t packet[1 + BINDING_UID_SIZE + DEVICE_ID_SIZE + PAIRING_KEY_SIZE];
    packet[0] = MSG_PAIRING;
    memcpy(packet + 1, bindingUID, BINDING_UID_SIZE);
    memcpy(packet + 1 + BINDING_UID_SIZE, txDeviceId, DEVICE_ID_SIZE);
    memcpy(packet + 1 + BINDING_UID_SIZE + DEVICE_ID_SIZE, key, PAIRING_KEY_SIZE);
    
    if (radio->send(packet, sizeof(packet))) {
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 2000) {
            Serial.print("[TX] Sent pairing packet (UID + TX ID + key), UID: ");
            for (int i = 0; i < BINDING_UID_SIZE; i++) {
                if (bindingUID[i] < 0x10) Serial.print("0");
                Serial.print(bindingUID[i], HEX);
            }
            Serial.println();
            lastLog = millis();
        }
    }
}

// ============================================================
// RX Side: Pairing mode management
// ============================================================
void Pairing::enterPairingMode(bool* pairingMode, unsigned long* pairingModeStartTime) {
    if (!pairingMode || !pairingModeStartTime) return;
    
    *pairingMode = true;
    *pairingModeStartTime = millis();
    
    Serial.println("*** PAIRING MODE ACTIVE ***");
    Serial.println("Waiting for TX to send pairing key...");
    Serial.print("Timeout in ");
    Serial.print(PAIRING_MODE_TIMEOUT_MS / 1000);
    Serial.println(" seconds");
}

void Pairing::exitPairingMode(bool* pairingMode, bool success) {
    if (!pairingMode) return;
    
    *pairingMode = false;
    
    if (success) {
        Serial.println("*** PAIRING SUCCESSFUL ***");
    } else {
        Serial.println("*** PAIRING MODE TIMEOUT ***");
    }
}

bool Pairing::getBootselButton() {
    return readBootselButton();
}

void Pairing::checkBootselButton(bool* pairingMode, unsigned long* pairingModeStartTime, bool isPaired) {
    if (!pairingMode || !pairingModeStartTime) return;
    
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck < 50) return;
    lastCheck = millis();
    
    static unsigned long buttonPressStart = 0;
    static bool buttonWasPressed = false;
    
    bool buttonPressed = getBootselButton();
    
    if (buttonPressed && !buttonWasPressed) {
        buttonPressStart = millis();
        buttonWasPressed = true;
        if (!isPaired) {
            Serial.println("[PAIR] BOOTSEL pressed (waiting for hold to enter pairing)");
        }
    } else if (buttonPressed && buttonWasPressed) {
        unsigned long holdTime = millis() - buttonPressStart;
        if (holdTime > BOOTSEL_HOLD_TIME_MS && !*pairingMode) {
            if (!isPaired) {
                Serial.println("[PAIR] BOOTSEL hold detected, entering pairing mode");
            }
            enterPairingMode(pairingMode, pairingModeStartTime);
        }
    } else if (!buttonPressed && buttonWasPressed) {
        buttonWasPressed = false;
        if (!isPaired) {
            Serial.println("[PAIR] BOOTSEL released");
        }
    }
}

void Pairing::updatePairingMode(bool* pairingMode, unsigned long pairingModeStartTime) {
    if (!pairingMode) return;
    if (!*pairingMode) return;
    if (millis() - pairingModeStartTime > PAIRING_MODE_TIMEOUT_MS) {
        exitPairingMode(pairingMode, false);
        return;
    }
}

// ============================================================
// RX Side: Handle pairing packet from TX
// ============================================================
bool Pairing::handlePairingPacket(const uint8_t* payload, size_t payloadLen,
                                   Security* security, SX128xLink* radio,
                                   const uint8_t* rxDeviceId, uint8_t* pairedTxDeviceId,
                                   bool* hasPairedTxId, bool* pairingMode) {
    if (!payload || !security || !radio || !rxDeviceId || !pairedTxDeviceId || 
        !hasPairedTxId || !pairingMode) {
        return false;
    }
    
    // Pairing packet format: [BINDING_UID: 8][TX_DEVICE_ID: 8][PAIRING_KEY: 16] = 32 bytes
    if (!*pairingMode || payloadLen < BINDING_UID_SIZE + DEVICE_ID_SIZE + PAIRING_KEY_SIZE) {
        return false;
    }
    
    // Extract binding UID
    uint8_t receivedUID[BINDING_UID_SIZE];
    memcpy(receivedUID, payload, BINDING_UID_SIZE);
    
    // CRITICAL: Verify binding UID matches before accepting pairing
    if (!security->verifyBindingUID(receivedUID)) {
        static unsigned long lastError = 0;
        if (millis() - lastError > 5000) {
            Serial.println("[RX] Pairing rejected - binding UID mismatch!");
            Serial.print("[RX] Received UID: ");
            for (int i = 0; i < BINDING_UID_SIZE; i++) {
                if (receivedUID[i] < 0x10) Serial.print("0");
                Serial.print(receivedUID[i], HEX);
            }
            Serial.println();
            lastError = millis();
        }
        return false;  // Reject pairing - UID doesn't match
    }
    
    // UID matches - proceed with pairing
    Serial.println("[RX] Binding UID verified - accepting pairing");
    
    // Extract TX device ID
    uint8_t txId[DEVICE_ID_SIZE];
    memcpy(txId, payload + BINDING_UID_SIZE, DEVICE_ID_SIZE);
    
    // Extract pairing key
    const uint8_t* key = payload + BINDING_UID_SIZE + DEVICE_ID_SIZE;
    
    // Store pairing key and TX device ID
    if (security->setPairingKey(key) && security->setPairedDeviceId(txId)) {
        memcpy(pairedTxDeviceId, txId, DEVICE_ID_SIZE);
        *hasPairedTxId = true;
        Serial.println("[RX] Pairing key and TX device ID saved");

        // Debug: print a short fingerprint of the pairing key so TX/RX can be compared
        // without printing the raw key. (Truncated HMAC over a constant label.)
        if (security->hasPairingKey()) {
            const uint8_t label[] = {'F','P','V','K','E','Y'};
            uint8_t fp[HMAC_SIZE];
            security->calculateHMAC(label, sizeof(label), fp);
            Serial.print("[RX] Pairing key fingerprint: ");
            for (int i = 0; i < HMAC_SIZE; i++) {
                if (fp[i] < 0x10) Serial.print("0");
                Serial.print(fp[i], HEX);
            }
            Serial.println();
        }
        
        // Send pairing ACK with RX device ID
        uint8_t ackPacket[1 + DEVICE_ID_SIZE];
        ackPacket[0] = MSG_PAIRING_ACK;
        memcpy(ackPacket + 1, rxDeviceId, DEVICE_ID_SIZE);
        if (radio->send(ackPacket, sizeof(ackPacket))) {
            Serial.println("[RX] Sent pairing ACK with RX device ID");
        }
        
        exitPairingMode(pairingMode, true);
        return true;
    } else {
        Serial.println("[RX] Failed to save pairing key or device ID");
        return false;
    }
}

// ============================================================
// TX Side: Handle pairing ACK from RX
// ============================================================
bool Pairing::handlePairingAck(const uint8_t* payload, size_t payloadLen,
                                Security* security, uint8_t* pairedRxDeviceId,
                                bool* hasPairedRxId, bool* pairingMode) {
    if (!payload || !security || !pairedRxDeviceId || !hasPairedRxId || !pairingMode) {
        return false;
    }
    
    // RX acknowledges pairing with its device ID
    // Format: [RX_DEVICE_ID: 8][HMAC: 4] = 12 bytes (but we only need device ID)
    if (!*pairingMode || payloadLen < DEVICE_ID_SIZE) {
        return false;
    }
    
    uint8_t rxId[DEVICE_ID_SIZE];
    memcpy(rxId, payload, DEVICE_ID_SIZE);
    
    // Store RX device ID
    if (security->setPairedDeviceId(rxId)) {
        memcpy(pairedRxDeviceId, rxId, DEVICE_ID_SIZE);
        *hasPairedRxId = true;
        *pairingMode = false;
        Serial.println("[TX] Pairing successful - RX device ID stored");
        Serial.print("[TX] RX Device ID: ");
        for (int i = 0; i < DEVICE_ID_SIZE; i++) {
            if (rxId[i] < 0x10) Serial.print("0");
            Serial.print(rxId[i], HEX);
        }
        Serial.println();

        // Debug: print a short fingerprint of the pairing key so TX/RX can be compared.
        if (security->hasPairingKey()) {
            const uint8_t label[] = {'F','P','V','K','E','Y'};
            uint8_t fp[HMAC_SIZE];
            security->calculateHMAC(label, sizeof(label), fp);
            Serial.print("[TX] Pairing key fingerprint: ");
            for (int i = 0; i < HMAC_SIZE; i++) {
                if (fp[i] < 0x10) Serial.print("0");
                Serial.print(fp[i], HEX);
            }
            Serial.println();
        }
        return true;
    }
    
    return false;
}
