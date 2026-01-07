// TX Side using SX128x (RadioLib) - radio comms only.
// Receives channels from RC board via UART protocol.
#include <Arduino.h>
#include <RadioLib.h>
#include "Protocol.h"
#include "Security.h"
#include "SX128xLink.h"
#include "pairing.h"
#include "PacketHandler.h"
#include "UARTProtocol.h"

// Timing
static unsigned long lastDataSent = 0;
static const unsigned long CONNECTION_TIMEOUT_MS = 2000;  // Connection lost detection (radio link state)

// Application state
Security security;
SX128xLink radio;

// Channel values - initialize to safe low values (1000)
uint16_t channels[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
uint16_t sequenceNumber = 0;

// Connection state machine
ConnectionState connectionState = STATE_DISCONNECTED;
unsigned long lastSyncSent = 0;
unsigned long lastSyncAckReceived = 0;
uint16_t syncSequence = 0;
unsigned long connectionStartTime = 0;

// Sync reliability / hysteresis
// The link can be perfectly fine while we occasionally miss a SYNC_ACK due to half-duplex timing.
// Never stop control traffic because of a single missed SYNC_ACK.
static unsigned long channelHoldoffUntil = 0;
static uint16_t lastSyncSeqSent = 0;
static uint16_t lastSyncSeqAcked = 0;
static uint8_t syncAckMisses = 0;
static const unsigned long SYNC_ACK_LISTEN_WINDOW_MS = 10;
static const uint8_t SYNC_ACK_MISSES_BEFORE_LOST = 3;

// Device IDs
uint8_t txDeviceId[DEVICE_ID_SIZE];
uint8_t pairedRxDeviceId[DEVICE_ID_SIZE];
bool hasPairedRxId = false;

// Pairing state
bool pairingMode = false;
unsigned long lastPairingSend = 0;

// UART Protocol for RC communication
#ifndef UART_PROTOCOL_TX
#define UART_PROTOCOL_TX 8
#endif
#ifndef UART_PROTOCOL_RX
#define UART_PROTOCOL_RX 9
#endif
UARTProtocol uartProto(&Serial2);

// Telemetry from RX
static int16_t last_rssi = 0;
static float last_snr = 0;
static uint16_t rx_battery_mv = 0;
static uint8_t rx_battery_pct = 0;
static bool rx_battery_valid = false;
static unsigned long lastBatteryUpdate = 0;

// Telemetry transmission timing
static unsigned long lastTelemetrySent = 0;
static const unsigned long TELEMETRY_INTERVAL = 200;  // 5Hz

// Status transmission
static ConnectionState lastReportedState = STATE_DISCONNECTED;
static unsigned long lastStatusSent = 0;
static const unsigned long STATUS_INTERVAL = 1000;  // 1Hz

// Statistics
static uint32_t packetsReceived = 0;
static uint32_t packetsLost = 0;

// Forward declarations
static void updateConnectionState();
static bool isConnectionReady();
static bool isPaired();
static void detectConnectionLoss();
static void receiveUARTMessages();
static void sendTelemetryToRC();
static void sendStatusToRC();
static void handleUARTCommands(UARTMsgType cmd);

// UART Protocol callbacks
static void onChannelsReceived(const ChannelData* data);
static void onCommandReceived(UARTMsgType cmd);

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200); 

    Serial.println("=== FPV Transmitter (SX128x) ===");
    Serial.println("Receiving channels from RC board via UART");
    Serial.flush();

    // Initialize security (loads device ID and pairing key if exists)
    Serial.println("Initializing security...");
    Serial.flush();
    if (security.begin()) {
        Serial.println("Security initialized");
        
        // Get this device's ID
        if (security.getDeviceId(txDeviceId)) {
            Serial.print("TX Device ID: ");
            for (int i = 0; i < DEVICE_ID_SIZE; i++) {
                if (txDeviceId[i] < 0x10) Serial.print("0");
                Serial.print(txDeviceId[i], HEX);
            }
            Serial.println();
        }
        
        // Get binding UID
        uint8_t bindingUID[BINDING_UID_SIZE];
        if (security.getBindingUID(bindingUID)) {
            Serial.print("Binding UID: ");
            for (int i = 0; i < BINDING_UID_SIZE; i++) {
                if (bindingUID[i] < 0x10) Serial.print("0");
                Serial.print(bindingUID[i], HEX);
            }
            Serial.print(" (from phrase: ");
            Serial.print(DEFAULT_BINDING_PHRASE);
            Serial.println(")");
        } else {
            Serial.println("WARNING: No binding UID - pairing will fail!");
        }
        
        // Check if we have a paired RX device ID
        if (security.getPairedDeviceId(pairedRxDeviceId)) {
            hasPairedRxId = true;
            Serial.print("Paired RX Device ID: ");
            for (int i = 0; i < DEVICE_ID_SIZE; i++) {
                if (pairedRxDeviceId[i] < 0x10) Serial.print("0");
                Serial.print(pairedRxDeviceId[i], HEX);
            }
            Serial.println();
        }
        
        // Check pairing status
        if (isPaired()) {
            Serial.println("Already paired - ready to connect");
            connectionState = STATE_CONNECTING;
        } else {
            Serial.println("Not paired - entering pairing mode");
            Serial.println("Will send pairing packets with binding UID");
            pairingMode = true;
            connectionState = STATE_PAIRING;
        }
    } else {
        Serial.println("Security init failed");
        connectionState = STATE_DISCONNECTED;
    }
    Serial.flush();

    // Initialize radio
    Serial.println("Initializing radio...");
    Serial.flush();
    bool radio_ready = false;
    if (radio.begin()) {
        Serial.println("SX128x ready");
        radio_ready = true;
    } else {
        Serial.println("SX128x init failed");
    }
    Serial.flush();

    // Initialize UART protocol for RC communication
    Serial.println("Initializing UART protocol...");
    Serial2.setTX(UART_PROTOCOL_TX);
    Serial2.setRX(UART_PROTOCOL_RX);
    uartProto.begin(UART_PROTOCOL_BAUDRATE);
    uartProto.setOnChannels(onChannelsReceived);
    uartProto.setOnCommand(onCommandReceived);
    Serial.print("UART Protocol on Serial2: TX=GP");
    Serial.print(UART_PROTOCOL_TX);
    Serial.print(", RX=GP");
    Serial.println(UART_PROTOCOL_RX);

    Serial.println("=== TX Ready ===");
    if (radio_ready) {
        Serial.println("Radio ready - waiting for channels from RC board");
    } else {
        Serial.println("WARNING: Radio not ready - cannot send channels");
    }
    Serial.flush();
}

// ============================================================
// Loop
// ============================================================
void loop() {
    // Process UART messages from RC board
    receiveUARTMessages();
    
    // Update connection state machine
    updateConnectionState();
    
    // Handle pairing if needed
    if (pairingMode) {
        Pairing::sendPairingKey(&radio, &security, txDeviceId, &pairingMode, &lastPairingSend);
    }
    
    // Send sync packets if connecting/connected.
    // If we just sent a sync packet, briefly pause channel TX to give the RX time to respond with SYNC_ACK.
    if (connectionState == STATE_CONNECTING || connectionState == STATE_CONNECTED) {
        unsigned long prevLastSyncSent = lastSyncSent;
        uint16_t prevSyncSeq = syncSequence;
        PacketHandler::sendSyncPacket(&radio, &security, txDeviceId, &syncSequence, &lastSyncSent);

        if (lastSyncSent != prevLastSyncSent && syncSequence != prevSyncSeq) {
            // We sent a new sync packet. If the previous one wasn't ACKed, count it.
            if (lastSyncSeqSent != 0 && lastSyncSeqAcked != lastSyncSeqSent) {
                if (syncAckMisses < 255) syncAckMisses++;
            }

            lastSyncSeqSent = syncSequence;
            channelHoldoffUntil = millis() + SYNC_ACK_LISTEN_WINDOW_MS;
        }
    }
    
    // Send channel data over radio (continuous when paired).
    // Do NOT gate control traffic on connection state; state is for UI/telemetry.
    // Briefly pause around sync TX to improve SYNC_ACK reception.
    if (isPaired() && !pairingMode && millis() >= channelHoldoffUntil) {
        PacketHandler::sendChannelData(&radio, &security, txDeviceId, channels, 
                                       &sequenceNumber, &lastDataSent, &last_rssi);
    }

    // Handle radio RX (telemetry, pairing ACK, sync ACK)
    SX128xPacket pkt;
    if (radio.receive(pkt) && pkt.length >= 1) {
        last_rssi = pkt.rssi;
        last_snr = pkt.snr;
        packetsReceived++;
        
        uint8_t type = pkt.data[0];
        const uint8_t* payload = pkt.data + 1;
        size_t payload_len = pkt.length - 1;
        
        switch (type) {
            case MSG_BATTERY:
                // Handle battery telemetry
                if (isPaired() && hasPairedRxId && payload_len >= 5) {
                    uint16_t voltage_x10 = (payload[0] << 8) | payload[1];
                    uint16_t current_x10 = (payload[2] << 8) | payload[3];
                    float rx_battery_voltage = voltage_x10 / 10.0f;
                    float rx_battery_current = current_x10 / 10.0f;
                    uint8_t rx_battery_remaining = payload[4];
                    
                    rx_battery_mv = (uint16_t)(rx_battery_voltage * 1000);
                    rx_battery_pct = rx_battery_remaining;
                    rx_battery_valid = true;
                    lastBatteryUpdate = millis();
                    
                    Serial.print("[TELEM] RX Batt ");
                    Serial.print(rx_battery_voltage, 1);
                    Serial.print("V ");
                    Serial.print(rx_battery_remaining);
                    Serial.println("%");
                }
                break;
            case MSG_PAIRING_ACK:
                if (pairingMode) {
                    if (Pairing::handlePairingAck(payload, payload_len, &security, 
                                                   pairedRxDeviceId, &hasPairedRxId, &pairingMode)) {
                        connectionState = STATE_CONNECTING;
                    }
                }
                break;
            case MSG_SYNC_ACK:
                if (isPaired() && hasPairedRxId) {
                    uint16_t syncSeq = 0;
                    uint32_t timestamp = 0;
                    uint8_t sourceDeviceId[DEVICE_ID_SIZE];
                    
                    if (Protocol::parseSyncAck(payload, payload_len, &syncSeq, &timestamp,
                                                &security, sourceDeviceId)) {
                        if (memcmp(sourceDeviceId, pairedRxDeviceId, DEVICE_ID_SIZE) == 0) {
                            lastSyncAckReceived = millis();
                            lastSyncSeqAcked = syncSeq;
                            syncAckMisses = 0;
                        } else {
                            static unsigned long lastIdMismatchLog = 0;
                            if (millis() - lastIdMismatchLog > 2000) {
                                lastIdMismatchLog = millis();
                                Serial.print("[SYNC] RX ID mismatch in SYNC_ACK (len=");
                                Serial.print(payload_len);
                                Serial.print(") src=");
                                for (int i = 0; i < DEVICE_ID_SIZE; i++) {
                                    if (sourceDeviceId[i] < 0x10) Serial.print("0");
                                    Serial.print(sourceDeviceId[i], HEX);
                                }
                                Serial.print(" expected=");
                                for (int i = 0; i < DEVICE_ID_SIZE; i++) {
                                    if (pairedRxDeviceId[i] < 0x10) Serial.print("0");
                                    Serial.print(pairedRxDeviceId[i], HEX);
                                }
                                Serial.println();
                            }
                        }
                    } else {
                        static unsigned long lastParseFailLog = 0;
                        if (millis() - lastParseFailLog > 2000) {
                            lastParseFailLog = millis();
                            Serial.print("[SYNC] parseSyncAck FAILED payloadLen=");
                            Serial.print(payload_len);
                            Serial.print(" keyLoaded=");
                            Serial.println(security.hasPairingKey() ? "Y" : "N");
                        }
                    }
                }
                break;
        }
    }
    
    // Send telemetry to RC board
    sendTelemetryToRC();
    
    // Send status updates to RC board
    sendStatusToRC();
    
    // Detect connection loss
    detectConnectionLoss();
}

// ============================================================
// UART Protocol - Message Reception
// ============================================================
void receiveUARTMessages() {
    uartProto.loop();
}

// ============================================================
// UART Protocol - Callbacks
// ============================================================
void onChannelsReceived(const ChannelData* data) {
    if (data) {
        for (int i = 0; i < 8; i++) {
            channels[i] = data->channels[i];
        }
    }
}

void onCommandReceived(UARTMsgType cmd) {
    handleUARTCommands(cmd);
}

// ============================================================
// UART Protocol - Command Handlers
// ============================================================
void handleUARTCommands(UARTMsgType cmd) {
    switch (cmd) {
        case UART_MSG_CMD_PAIR:
            Serial.println("[UART] PAIR command received");
            pairingMode = true;
            connectionState = STATE_PAIRING;
            uartProto.sendAck(UART_MSG_CMD_PAIR);
            break;
            
        case UART_MSG_CMD_BOND:
            Serial.println("[UART] BOND command received");
            if (isPaired() && hasPairedRxId) {
                // Already bonded
                uartProto.sendAck(UART_MSG_CMD_BOND);
            } else {
                uartProto.sendError(1);  // Not paired
            }
            break;
            
        case UART_MSG_CMD_RESTART:
            Serial.println("[UART] RESTART command received");
            uartProto.sendAck(UART_MSG_CMD_RESTART);
            delay(100);
            rp2040.restart();
            break;
            
        case UART_MSG_CMD_STATUS_REQ:
            Serial.println("[UART] STATUS_REQ command received");
            sendStatusToRC();  // Send status immediately
            uartProto.sendAck(UART_MSG_CMD_STATUS_REQ);
            break;
            
        default:
            break;
    }
}

// ============================================================
// UART Protocol - Telemetry Transmission
// ============================================================
void sendTelemetryToRC() {
    if (millis() - lastTelemetrySent < TELEMETRY_INTERVAL) return;
    lastTelemetrySent = millis();
    
    TelemetryData telem;
    telem.rssi = last_rssi;
    telem.snr = last_snr;
    
    if (rx_battery_valid && (millis() - lastBatteryUpdate) < 5000) {
        telem.rxBattMv = rx_battery_mv;
        telem.rxBattPct = rx_battery_pct;
    } else {
        telem.rxBattMv = 0;
        telem.rxBattPct = 0;
    }
    
    // Calculate link quality (0-100%) based on RSSI
    // RSSI typically ranges from -120 to -30 dBm
    // Map to 0-100%
    if (last_rssi > -50) {
        telem.linkQuality = 100;
    } else if (last_rssi < -120) {
        telem.linkQuality = 0;
    } else {
        telem.linkQuality = (uint8_t)map(last_rssi, -120, -50, 0, 100);
    }
    
    uartProto.sendTelemetry(&telem);
}

// ============================================================
// UART Protocol - Status Transmission
// ============================================================
void sendStatusToRC() {
    // Send status when state changes or periodically
    bool stateChanged = (connectionState != lastReportedState);
    bool periodicUpdate = (millis() - lastStatusSent) > STATUS_INTERVAL;
    
    if (!stateChanged && !periodicUpdate) return;
    
    StatusData status;
    status.connectionState = (uint8_t)connectionState;
    status.pairingState = isPaired() ? 1 : 0;
    status.packetsReceived = packetsReceived;
    status.packetsLost = packetsLost;
    
    uartProto.sendStatus(&status);
    
    lastReportedState = connectionState;
    lastStatusSent = millis();
}

// ============================================================
// Connection state management
// ============================================================
bool isPaired() {
    return security.hasPairingKey() && hasPairedRxId;
}

bool isConnectionReady() {
    return isPaired() && connectionState == STATE_CONNECTED && 
           (lastSyncAckReceived > 0 && (millis() - lastSyncAckReceived < CONNECTION_TIMEOUT_MS));
}

void updateConnectionState() {
    static unsigned long lastStateLog = 0;
    
    // State transitions
    switch (connectionState) {
        case STATE_DISCONNECTED:
            if (pairingMode) {
                connectionState = STATE_PAIRING;
                Serial.println("[STATE] DISCONNECTED -> PAIRING");
            }
            break;
            
        case STATE_PAIRING:
            if (!pairingMode && isPaired()) {
                connectionState = STATE_CONNECTING;
                Serial.println("[STATE] PAIRING -> CONNECTING");
            }
            break;
            
        case STATE_CONNECTING:
            // Wait for sync ACK to complete handshake
            if (lastSyncAckReceived > 0 && (millis() - lastSyncAckReceived < 1000)) {
                connectionState = STATE_CONNECTED;
                connectionStartTime = millis();
                Serial.println("[STATE] CONNECTING -> CONNECTED");
            }
            break;
            
        case STATE_CONNECTED:
            // Stay connected as long as we receive sync ACKs
            break;
            
        case STATE_LOST:
            // Try to reconnect
            if (isPaired()) {
                connectionState = STATE_CONNECTING;
                Serial.println("[STATE] LOST -> CONNECTING (reconnecting)");
            } else {
                connectionState = STATE_DISCONNECTED;
                pairingMode = true;
                Serial.println("[STATE] LOST -> DISCONNECTED (need pairing)");
            }
            break;
    }
    
    // Log state periodically
    if (millis() - lastStateLog > 10000) {
        const char* stateNames[] = {"DISCONNECTED", "PAIRING", "CONNECTING", "CONNECTED", "LOST"};
        Serial.print("[STATE] Current: ");
        Serial.print(stateNames[connectionState]);
        if (connectionState == STATE_CONNECTED) {
            Serial.print(" (RSSI: ");
            Serial.print(last_rssi);
            Serial.print(", SNR: ");
            Serial.print(last_snr);
            Serial.print(")");
        }
        Serial.println();
        lastStateLog = millis();
    }
}

void detectConnectionLoss() {
    if (connectionState == STATE_CONNECTED) {
        // Use hysteresis: only mark LOST after multiple consecutive missed SYNC_ACKs.
        // This avoids false disconnects caused by half-duplex timing (e.g., channel TX colliding with ACK).
        if (syncAckMisses >= SYNC_ACK_MISSES_BEFORE_LOST) {
            connectionState = STATE_LOST;
            packetsLost++;
            Serial.println("[STATE] CONNECTED -> LOST (missed sync ACKs)");
        }
    }
}

