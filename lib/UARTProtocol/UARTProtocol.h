#pragma once

#include <Arduino.h>
#include "../crc8/crc8.h"

// UART Protocol Message Types
enum UARTMsgType : uint8_t {
    UART_MSG_CHANNELS = 0x01,
    UART_MSG_CMD_PAIR = 0x10,
    UART_MSG_CMD_BOND = 0x11,
    UART_MSG_CMD_RESTART = 0x12,
    UART_MSG_CMD_STATUS_REQ = 0x13,
    UART_MSG_PING = 0x14,      // Device availability check
    UART_MSG_PONG = 0x15,      // Device availability response
    UART_MSG_TELEMETRY = 0x20,
    UART_MSG_STATUS = 0x21,
    UART_MSG_ACK = 0x22,
    UART_MSG_ERROR = 0x23
};

// Protocol constants
#define UART_PROTOCOL_SYNC_BYTE 0xA5
#define UART_PROTOCOL_MAX_PAYLOAD 60
#define UART_PROTOCOL_MAX_FRAME_SIZE (1 + 1 + 1 + UART_PROTOCOL_MAX_PAYLOAD + 1)  // SYNC + LEN + TYPE + PAYLOAD + CRC
#define UART_PROTOCOL_BAUDRATE 420000

// Data structures
struct ChannelData {
    uint16_t channels[8];  // 1000-2000µs
} __attribute__((packed));

struct TelemetryData {
    int16_t rssi;          // dBm
    float snr;             // dB
    uint16_t rxBattMv;     // millivolts
    uint8_t rxBattPct;     // percentage
    uint8_t linkQuality;   // 0-100%
} __attribute__((packed));

struct StatusData {
    uint8_t connectionState;  // ConnectionState enum
    uint8_t pairingState;     // 0=unpaired, 1=paired
    uint32_t packetsReceived;
    uint32_t packetsLost;
} __attribute__((packed));

// Callback function types
typedef void (*OnChannelsCallback)(const ChannelData* data);
typedef void (*OnTelemetryCallback)(const TelemetryData* data);
typedef void (*OnStatusCallback)(const StatusData* data);
typedef void (*OnCommandCallback)(UARTMsgType cmd);
typedef void (*OnAckCallback)(UARTMsgType ackedCmd);
typedef void (*OnErrorCallback)(uint8_t errorCode);
typedef void (*OnPongCallback)();  // PONG received (device is alive)

class UARTProtocol {
public:
    UARTProtocol(HardwareSerial* serial);
    
    // Initialization
    void begin(uint32_t baud = UART_PROTOCOL_BAUDRATE);
    
    // Send methods
    bool sendChannels(const uint16_t channels[8]);
    bool sendTelemetry(const TelemetryData* data);
    bool sendStatus(const StatusData* data);
    bool sendCommand(UARTMsgType cmd);
    bool sendAck(UARTMsgType ackedCmd);
    bool sendError(uint8_t errorCode);
    bool sendPing();  // Device availability check
    bool sendPong();  // Device availability response
    
    // Main loop - call this regularly to process incoming messages
    void loop();
    
    // Callback setters
    void setOnChannels(OnChannelsCallback cb) { onChannels = cb; }
    void setOnTelemetry(OnTelemetryCallback cb) { onTelemetry = cb; }
    void setOnStatus(OnStatusCallback cb) { onStatus = cb; }
    void setOnCommand(OnCommandCallback cb) { onCommand = cb; }
    void setOnAck(OnAckCallback cb) { onAck = cb; }
    void setOnError(OnErrorCallback cb) { onError = cb; }
    void setOnPong(OnPongCallback cb) { onPong = cb; }
    
    // Statistics
    uint32_t getPacketsSent() const { return packetsSent; }
    uint32_t getPacketsReceived() const { return packetsReceived; }
    uint32_t getPacketsDropped() const { return packetsDropped; }
    void resetStats() { packetsSent = 0; packetsReceived = 0; packetsDropped = 0; }

private:
    HardwareSerial* serial;
    Crc8 crc8;
    
    // RX state machine
    enum RxState {
        RX_STATE_SYNC,
        RX_STATE_LENGTH,
        RX_STATE_TYPE,
        RX_STATE_PAYLOAD,
        RX_STATE_CRC
    };
    
    RxState rxState;
    uint8_t rxBuffer[UART_PROTOCOL_MAX_PAYLOAD];
    uint8_t rxBufferPos;
    uint8_t rxExpectedLength;
    UARTMsgType rxMessageType;
    unsigned long rxTimeout;
    static const unsigned long RX_TIMEOUT_MS = 100;
    
    // Statistics
    uint32_t packetsSent;
    uint32_t packetsReceived;
    uint32_t packetsDropped;
    
    // Callbacks
    OnChannelsCallback onChannels;
    OnTelemetryCallback onTelemetry;
    OnStatusCallback onStatus;
    OnCommandCallback onCommand;
    OnAckCallback onAck;
    OnErrorCallback onError;
    OnPongCallback onPong;
    
    // Internal methods
    void processRxByte(uint8_t byte);
    void resetRxState();
    void processMessage(UARTMsgType type, const uint8_t* payload, uint8_t length);
    bool sendFrame(UARTMsgType type, const uint8_t* payload, uint8_t payloadLength);
    uint8_t calculateCRC(uint8_t length, uint8_t type, const uint8_t* payload, uint8_t payloadLength);
};
