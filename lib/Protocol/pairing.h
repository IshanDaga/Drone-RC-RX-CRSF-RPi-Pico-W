#ifndef PAIRING_H
#define PAIRING_H

#include <Arduino.h>
#include "Protocol.h"

// Forward declarations
class Security;
class SX128xLink;

// Pairing mode configuration
#define PAIRING_MODE_TIMEOUT_MS 60000   // 60 seconds pairing window
#define BOOTSEL_HOLD_TIME_MS    5000    // 5 seconds to enter pairing mode
#define PAIRING_SCAN_INTERVAL_MS 500    // How often to send pairing packets

class Pairing {
public:
    // TX side: Send pairing key packet
    static void sendPairingKey(SX128xLink* radio, Security* security, 
                               const uint8_t* txDeviceId, bool* pairingMode, 
                               unsigned long* lastPairingSend);
    
    // RX side: Enter pairing mode
    static void enterPairingMode(bool* pairingMode, unsigned long* pairingModeStartTime);
    
    // RX side: Exit pairing mode
    static void exitPairingMode(bool* pairingMode, bool success);
    
    // RX side: Check BOOTSEL button (RP2040-specific)
    static bool getBootselButton();
    
    // RX side: Check if BOOTSEL button is held to enter pairing
    // Logs button press activity when not paired
    static void checkBootselButton(bool* pairingMode, unsigned long* pairingModeStartTime, bool isPaired);
    
    // RX side: Update pairing mode timeout
    static void updatePairingMode(bool* pairingMode, unsigned long pairingModeStartTime);
    
    // RX side: Handle pairing packet from TX
    static bool handlePairingPacket(const uint8_t* payload, size_t payloadLen,
                                    Security* security, SX128xLink* radio,
                                    const uint8_t* rxDeviceId, uint8_t* pairedTxDeviceId,
                                    bool* hasPairedTxId, bool* pairingMode);
    
    // TX side: Handle pairing ACK from RX
    static bool handlePairingAck(const uint8_t* payload, size_t payloadLen,
                                  Security* security, uint8_t* pairedRxDeviceId,
                                  bool* hasPairedRxId, bool* pairingMode);
};

#endif // PAIRING_H
