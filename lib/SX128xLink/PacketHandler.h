#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include <Arduino.h>
#include "SX128xLink.h"
#include "Protocol.h"

// Forward declarations
class Security;

// Timing constants
#define DATA_INTERVAL_MS 6  // ~167Hz (higher than 150Hz for safety margin)
#define SYNC_INTERVAL_MS 1500
#define BATTERY_TX_INTERVAL_MS 10000

class PacketHandler {
public:
    // TX side: Send channel data
    static void sendChannelData(SX128xLink* radio, Security* security,
                                 const uint8_t* txDeviceId, uint16_t* channels,
                                 uint16_t* sequenceNumber, unsigned long* lastDataSent,
                                 int16_t* lastRssi);
    
    // TX side: Handle radio RX (telemetry from RX)
    static void handleRadioRxTx(SX128xLink* radio, Security* security,
                                 const uint8_t* pairedRxDeviceId, bool hasPairedRxId,
                                 int16_t* lastRssi, float* lastSnr);
    
    // TX side: Send sync packet
    static void sendSyncPacket(SX128xLink* radio, Security* security,
                               const uint8_t* txDeviceId, uint16_t* syncSequence,
                               unsigned long* lastSyncSent);
    
    // RX side: Handle radio RX (channels from TX)
    static bool handleRadioRxRx(SX128xLink* radio, Security* security,
                                 const uint8_t* pairedTxDeviceId, bool hasPairedTxId,
                                 uint16_t* channels, uint16_t* lastSequence,
                                 unsigned long* lastDataReceived,
                                 int16_t* lastRSSI, float* lastSNR,
                                 ConnectionState connectionState);
    
    // RX side: Handle sync packet
    static bool handleSyncPacket(const uint8_t* packet, size_t len,
                                  Security* security, const uint8_t* pairedTxDeviceId,
                                  bool hasPairedTxId, uint16_t* syncSequence,
                                  unsigned long* lastSyncReceived);
    
    // RX side: Send sync ACK
    static void sendSyncAck(SX128xLink* radio, Security* security,
                            const uint8_t* rxDeviceId, uint16_t syncSeq);
    
    // RX side: Send battery telemetry
    static void sendBatteryTelemetry(SX128xLink* radio, float batteryVoltage,
                                      float batteryCurrent, uint8_t batteryRemaining,
                                      bool batteryDataValid,
                                      unsigned long* lastBatteryTxTime);
};

#endif // PACKET_HANDLER_H
