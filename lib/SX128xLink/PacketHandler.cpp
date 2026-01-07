#include "PacketHandler.h"
#include "Security.h"
#include "Protocol.h"
#include <string.h>

// ============================================================
// TX Side: Send channel data
// ============================================================
void PacketHandler::sendChannelData(SX128xLink* radio, Security* security,
                                     const uint8_t* txDeviceId, uint16_t* channels,
                                     uint16_t* sequenceNumber, unsigned long* lastDataSent,
                                     int16_t* lastRssi) {
    if (!radio || !security || !txDeviceId || !channels || !sequenceNumber || 
        !lastDataSent || !lastRssi) {
        return;
    }
    
    // Only send if radio is ready
    if (!radio->isReady()) {
        return;
    }
    
    if (millis() - *lastDataSent < DATA_INTERVAL_MS) return;
    *lastDataSent = millis();
    
    (*sequenceNumber)++;
    if (*sequenceNumber == 0) *sequenceNumber = 1;
    
    // Encode frame with device ID and encryption
    uint8_t frame[FRAME_SIZE];
    Protocol::encodeFrame(channels, *sequenceNumber, security, txDeviceId, frame);
    
    uint8_t packet[1 + FRAME_SIZE];
    packet[0] = MSG_CHANNELS;
    memcpy(packet + 1, frame, FRAME_SIZE);
    
    if (radio->send(packet, sizeof(packet))) {
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 2000) {
            Serial.print("[TX] Channels: ");
            for (int i = 0; i < 4; i++) {
                Serial.print(channels[i]);
                if (i < 3) Serial.print(",");
            }
            Serial.print(" Seq:");
            Serial.print(*sequenceNumber);
            Serial.print(" RSSI:");
            Serial.print(*lastRssi);
            Serial.println();
            lastLog = millis();
        }
    }
}

// ============================================================
// TX Side: Handle radio RX (telemetry from RX)
// ============================================================
void PacketHandler::handleRadioRxTx(SX128xLink* radio, Security* security,
                                     const uint8_t* pairedRxDeviceId, bool hasPairedRxId,
                                     int16_t* lastRssi, float* lastSnr) {
    if (!radio || !security || !pairedRxDeviceId || !lastRssi || !lastSnr) {
        return;
    }
    
    SX128xPacket pkt;
    if (!radio->receive(pkt)) {
        return;
    }
    
    if (pkt.length < 1) {
        return;
    }
    
    *lastRssi = pkt.rssi;
    *lastSnr = pkt.snr;
    
    uint8_t type = pkt.data[0];
    const uint8_t* payload = pkt.data + 1;
    size_t payload_len = pkt.length - 1;
    
    switch (type) {
        case MSG_BATTERY:
            // Only accept battery telemetry from paired RX (device ID filtering)
            if (!hasPairedRxId) {
                return;
            }
            // Note: Battery telemetry doesn't include device ID in current format
            // For now, accept it if we're paired (could add device ID to battery packet later)
            if (payload_len >= 5) {
                uint16_t voltage_x10 = (payload[0] << 8) | payload[1];
                uint16_t current_x10 = (payload[2] << 8) | payload[3];
                float rx_battery_voltage = voltage_x10 / 10.0f;
                float rx_battery_current = current_x10 / 10.0f;
                uint8_t rx_battery_remaining = payload[4];
                Serial.print("[TELEM] RX Batt ");
                Serial.print(rx_battery_voltage, 1);
                Serial.print("V ");
                Serial.print(rx_battery_remaining);
                Serial.println("%");
            }
            break;
        case MSG_SYNC_ACK:
            // Handle sync ACK from RX
            if (hasPairedRxId) {
                uint16_t syncSeq = 0;
                uint32_t timestamp = 0;
                uint8_t sourceDeviceId[DEVICE_ID_SIZE];
                
                if (Protocol::parseSyncAck(payload, payload_len, &syncSeq, &timestamp, security, sourceDeviceId)) {
                    // Verify device ID matches paired RX
                    if (memcmp(sourceDeviceId, pairedRxDeviceId, DEVICE_ID_SIZE) == 0) {
                        static unsigned long lastSyncAckLog = 0;
                        if (millis() - lastSyncAckLog > 5000) {
                            Serial.print("[SYNC] Received sync ACK seq: ");
                            Serial.println(syncSeq);
                            lastSyncAckLog = millis();
                        }
                    }
                }
            }
            break;
        default:
            break;
    }
}

// ============================================================
// TX Side: Send sync packet
// ============================================================
void PacketHandler::sendSyncPacket(SX128xLink* radio, Security* security,
                                    const uint8_t* txDeviceId, uint16_t* syncSequence,
                                    unsigned long* lastSyncSent) {
    if (!radio || !security || !txDeviceId || !syncSequence || !lastSyncSent) {
        return;
    }
    
    if (!radio->isReady()) {
        return;
    }
    
    // Send sync packet periodically
    if (millis() - *lastSyncSent < SYNC_INTERVAL_MS) {
        return;
    }
    *lastSyncSent = millis();
    
    (*syncSequence)++;
    if (*syncSequence == 0) *syncSequence = 1;
    
    uint8_t packet[64];
    size_t packetLen = 0;
    uint32_t timestamp = millis();
    
    Protocol::createSyncPacket(*syncSequence, timestamp, security, txDeviceId, packet, &packetLen);
    
    // Prepend message type
    uint8_t fullPacket[65];
    fullPacket[0] = MSG_SYNC;
    memcpy(fullPacket + 1, packet, packetLen);
    
    radio->send(fullPacket, packetLen + 1);
}

// ============================================================
// RX Side: Handle radio RX (channels from TX)
// ============================================================
bool PacketHandler::handleRadioRxRx(SX128xLink* radio, Security* security,
                                      const uint8_t* pairedTxDeviceId, bool hasPairedTxId,
                                      uint16_t* channels, uint16_t* lastSequence,
                                      unsigned long* lastDataReceived,
                                      int16_t* lastRSSI, float* lastSNR,
                                      ConnectionState connectionState) {
    if (!radio || !security || !pairedTxDeviceId || !channels || !lastSequence ||
        !lastDataReceived || !lastRSSI || !lastSNR) {
        return false;
    }
    
    SX128xPacket pkt;
    if (!radio->receive(pkt)) {
        return false;
    }
    if (pkt.length < 1) {
        return false;
    }
    
    *lastRSSI = pkt.rssi;
    *lastSNR = pkt.snr;
    
    uint8_t type = pkt.data[0];
    const uint8_t* payload = pkt.data + 1;
    size_t payload_len = pkt.length - 1;
    
    if (type == MSG_CHANNELS) {
        // Accept channel data once paired. We allow CONNECTING as well so control
        // traffic doesn't stall if the sync handshake is jittery.
        if (!hasPairedTxId || (connectionState != STATE_CONNECTED && connectionState != STATE_CONNECTING)) {
            return false;
        }
        
        if (payload_len < FRAME_SIZE) {
            return false;
        }
        
        // Decode with device ID verification (only accept from paired TX)
        uint16_t sequence = 0;
        if (Protocol::decodeFrame(payload, channels, &sequence, security, lastSequence, pairedTxDeviceId)) {
            *lastDataReceived = millis();
            static unsigned long lastPrint = 0;
            if (millis() - lastPrint > 1000) {
                Serial.print("[RX] Channels: ");
                for (int i = 0; i < 8; i++) {
                    Serial.print(channels[i]);
                    Serial.print(" ");
                }
                Serial.print(" Seq: ");
                Serial.print(sequence);
                Serial.print(" RSSI: ");
                Serial.print(*lastRSSI);
                Serial.print(" SNR: ");
                Serial.println(*lastSNR);
                lastPrint = millis();
            }
            return true;
        } else {
            static unsigned long lastError = 0;
            if (millis() - lastError > 5000) {
                Serial.println("[RX] Invalid frame (device ID mismatch or security check failed)");
                lastError = millis();
            }
        }
    }
    
    return false;
}

// ============================================================
// RX Side: Handle sync packet
// ============================================================
bool PacketHandler::handleSyncPacket(const uint8_t* packet, size_t len,
                                      Security* security, const uint8_t* pairedTxDeviceId,
                                      bool hasPairedTxId, uint16_t* syncSequence,
                                      unsigned long* lastSyncReceived) {
    if (!packet || !security || !pairedTxDeviceId || !syncSequence || !lastSyncReceived) {
        return false;
    }
    
    if (!hasPairedTxId) {
        return false;
    }
    
    uint16_t syncSeq = 0;
    uint32_t timestamp = 0;
    uint8_t sourceDeviceId[DEVICE_ID_SIZE];
    
    if (Protocol::parseSyncPacket(packet, len, &syncSeq, &timestamp, security, sourceDeviceId)) {
        // Verify device ID matches paired TX
        if (memcmp(sourceDeviceId, pairedTxDeviceId, DEVICE_ID_SIZE) == 0) {
            *lastSyncReceived = millis();
            *syncSequence = syncSeq;
            
            static unsigned long lastSyncLog = 0;
            if (millis() - lastSyncLog > 5000) {
                Serial.print("[SYNC] Received sync seq: ");
                Serial.println(syncSeq);
                lastSyncLog = millis();
            }
            return true;
        } else {
            static unsigned long lastError = 0;
            if (millis() - lastError > 5000) {
                Serial.println("[SYNC] Device ID mismatch in sync packet");
                lastError = millis();
            }
        }
    } else {
        // Debug: sync parse failures can leave the system stuck in CONNECTING with no visibility.
        // Rate limit to avoid spamming serial.
        static unsigned long lastParseFailLog = 0;
        if (millis() - lastParseFailLog > 2000) {
            lastParseFailLog = millis();

            Serial.print("[SYNC] parseSyncPacket FAILED len=");
            Serial.print(len);
            Serial.print(" keyLoaded=");
            Serial.print(security && security->hasPairingKey() ? "Y" : "N");

            // If the packet is long enough, print type + HMAC comparison details.
            const size_t minLen = 1 + DEVICE_ID_SIZE + 2 + 4 + HMAC_SIZE;
            if (packet && len >= minLen) {
                const size_t hmacOffset = 1 + DEVICE_ID_SIZE + 2 + 4;

                Serial.print(" type=0x");
                Serial.print(packet[0], HEX);

                // Print source device ID (first 8 bytes after type)
                Serial.print(" srcId=");
                for (int i = 0; i < DEVICE_ID_SIZE; i++) {
                    if (packet[1 + i] < 0x10) Serial.print("0");
                    Serial.print(packet[1 + i], HEX);
                }

                if (security && security->hasPairingKey()) {
                    uint8_t calc[HMAC_SIZE];
                    security->calculateHMAC(packet, hmacOffset, calc);

                    Serial.print(" hmacCalc=");
                    for (int i = 0; i < HMAC_SIZE; i++) {
                        if (calc[i] < 0x10) Serial.print("0");
                        Serial.print(calc[i], HEX);
                    }
                    Serial.print(" hmacRx=");
                    for (int i = 0; i < HMAC_SIZE; i++) {
                        uint8_t b = packet[hmacOffset + i];
                        if (b < 0x10) Serial.print("0");
                        Serial.print(b, HEX);
                    }
                }
            }
            Serial.println();
        }
    }
    
    return false;
}

// ============================================================
// RX Side: Send sync ACK
// ============================================================
void PacketHandler::sendSyncAck(SX128xLink* radio, Security* security,
                                 const uint8_t* rxDeviceId, uint16_t syncSeq) {
    if (!radio || !security || !rxDeviceId) {
        return;
    }
    
    if (!radio->isReady()) {
        return;
    }
    
    uint8_t packet[64];
    size_t packetLen = 0;
    uint32_t timestamp = millis();
    
    Protocol::createSyncAck(syncSeq, timestamp, security, rxDeviceId, packet, &packetLen);
    
    // Prepend message type
    uint8_t fullPacket[65];
    fullPacket[0] = MSG_SYNC_ACK;
    memcpy(fullPacket + 1, packet, packetLen);
    
    radio->send(fullPacket, packetLen + 1);
}

// ============================================================
// RX Side: Send battery telemetry
// ============================================================
void PacketHandler::sendBatteryTelemetry(SX128xLink* radio, float batteryVoltage,
                                          float batteryCurrent, uint8_t batteryRemaining,
                                          bool batteryDataValid,
                                          unsigned long* lastBatteryTxTime) {
    if (!radio || !lastBatteryTxTime) {
        return;
    }
    
    if (!batteryDataValid) return;
    if (!radio->isReady()) return;
    if (millis() - *lastBatteryTxTime < BATTERY_TX_INTERVAL_MS) return;
    *lastBatteryTxTime = millis();
    
    uint8_t packet[1 + 5];
    uint16_t voltage_x10 = (uint16_t)(batteryVoltage * 10);
    uint16_t current_x10 = (uint16_t)(batteryCurrent * 10);
    packet[0] = MSG_BATTERY;
    packet[1] = (voltage_x10 >> 8) & 0xFF;
    packet[2] = voltage_x10 & 0xFF;
    packet[3] = (current_x10 >> 8) & 0xFF;
    packet[4] = current_x10 & 0xFF;
    packet[5] = batteryRemaining;
    
    if (radio->send(packet, sizeof(packet))) {
        Serial.print("[RADIO TELEM] Sent battery: ");
        Serial.print(batteryVoltage, 1);
        Serial.print("V ");
        Serial.print(batteryRemaining);
        Serial.println("%");
    }
}
