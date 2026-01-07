#include "UARTProtocol.h"
#include <string.h>

UARTProtocol::UARTProtocol(HardwareSerial* serial)
    : serial(serial)
    , crc8(0xD5)  // CRC8 polynomial for CRSF (same as used in crsfSerial)
    , rxState(RX_STATE_SYNC)
    , rxBufferPos(0)
    , rxExpectedLength(0)
    , rxMessageType(UART_MSG_CHANNELS)
    , rxTimeout(0)
    , packetsSent(0)
    , packetsReceived(0)
    , packetsDropped(0)
    , onChannels(nullptr)
    , onTelemetry(nullptr)
    , onStatus(nullptr)
    , onCommand(nullptr)
    , onAck(nullptr)
    , onError(nullptr)
    , onPong(nullptr)
{
}

void UARTProtocol::begin(uint32_t baud) {
    if (serial) {
        serial->begin(baud);
    }
    resetRxState();
}

void UARTProtocol::loop() {
    if (!serial) return;
    
    // Check for RX timeout
    if (rxState != RX_STATE_SYNC && (millis() - rxTimeout) > RX_TIMEOUT_MS) {
        packetsDropped++;
        resetRxState();
    }
    
    // Process incoming bytes
    while (serial->available()) {
        uint8_t byte = serial->read();
        processRxByte(byte);
    }
}

void UARTProtocol::processRxByte(uint8_t byte) {
    rxTimeout = millis();
    
    switch (rxState) {
        case RX_STATE_SYNC:
            if (byte == UART_PROTOCOL_SYNC_BYTE) {
                rxState = RX_STATE_LENGTH;
            }
            break;
            
        case RX_STATE_LENGTH:
            if (byte <= UART_PROTOCOL_MAX_PAYLOAD) {
                rxExpectedLength = byte;
                rxState = RX_STATE_TYPE;
            } else {
                // Invalid length
                packetsDropped++;
                resetRxState();
            }
            break;
            
        case RX_STATE_TYPE:
            rxMessageType = (UARTMsgType)byte;
            rxBufferPos = 0;
            if (rxExpectedLength == 0) {
                // No payload, go directly to CRC
                rxState = RX_STATE_CRC;
            } else {
                rxState = RX_STATE_PAYLOAD;
            }
            break;
            
        case RX_STATE_PAYLOAD:
            if (rxBufferPos < UART_PROTOCOL_MAX_PAYLOAD) {
                rxBuffer[rxBufferPos++] = byte;
                if (rxBufferPos >= rxExpectedLength) {
                    rxState = RX_STATE_CRC;
                }
            } else {
                // Buffer overflow
                packetsDropped++;
                resetRxState();
            }
            break;
            
        case RX_STATE_CRC: {
            // Verify CRC
            uint8_t calculatedCRC = calculateCRC(rxExpectedLength, (uint8_t)rxMessageType, rxBuffer, rxExpectedLength);
            if (byte == calculatedCRC) {
                // Valid message
                packetsReceived++;
                processMessage(rxMessageType, rxBuffer, rxExpectedLength);
            } else {
                // CRC mismatch
                packetsDropped++;
            }
            resetRxState();
            break;
        }
    }
}

void UARTProtocol::resetRxState() {
    rxState = RX_STATE_SYNC;
    rxBufferPos = 0;
    rxExpectedLength = 0;
    rxTimeout = millis();
}

void UARTProtocol::processMessage(UARTMsgType type, const uint8_t* payload, uint8_t length) {
    switch (type) {
        case UART_MSG_CHANNELS:
            if (length == sizeof(ChannelData) && onChannels) {
                onChannels((const ChannelData*)payload);
            }
            break;
            
        case UART_MSG_TELEMETRY:
            if (length == sizeof(TelemetryData) && onTelemetry) {
                onTelemetry((const TelemetryData*)payload);
            }
            break;
            
        case UART_MSG_STATUS:
            if (length == sizeof(StatusData) && onStatus) {
                onStatus((const StatusData*)payload);
            }
            break;
            
        case UART_MSG_CMD_PAIR:
        case UART_MSG_CMD_BOND:
        case UART_MSG_CMD_RESTART:
        case UART_MSG_CMD_STATUS_REQ:
            if (onCommand) {
                onCommand(type);
            }
            break;
            
        case UART_MSG_PING:
            // Respond to ping with pong
            sendPong();
            break;
            
        case UART_MSG_PONG:
            // Pong received - device is alive
            if (onPong) {
                onPong();
            }
            break;
            
        case UART_MSG_ACK:
            if (length == 1 && onAck) {
                onAck((UARTMsgType)payload[0]);
            }
            break;
            
        case UART_MSG_ERROR:
            if (length == 1 && onError) {
                onError(payload[0]);
            }
            break;
            
        default:
            // Unknown message type
            break;
    }
}

bool UARTProtocol::sendChannels(const uint16_t channels[8]) {
    ChannelData data;
    for (int i = 0; i < 8; i++) {
        data.channels[i] = channels[i];
    }
    return sendFrame(UART_MSG_CHANNELS, (const uint8_t*)&data, sizeof(ChannelData));
}

bool UARTProtocol::sendTelemetry(const TelemetryData* data) {
    if (!data) return false;
    return sendFrame(UART_MSG_TELEMETRY, (const uint8_t*)data, sizeof(TelemetryData));
}

bool UARTProtocol::sendStatus(const StatusData* data) {
    if (!data) return false;
    return sendFrame(UART_MSG_STATUS, (const uint8_t*)data, sizeof(StatusData));
}

bool UARTProtocol::sendCommand(UARTMsgType cmd) {
    // Commands have no payload
    return sendFrame(cmd, nullptr, 0);
}

bool UARTProtocol::sendAck(UARTMsgType ackedCmd) {
    uint8_t payload = (uint8_t)ackedCmd;
    return sendFrame(UART_MSG_ACK, &payload, 1);
}

bool UARTProtocol::sendError(uint8_t errorCode) {
    return sendFrame(UART_MSG_ERROR, &errorCode, 1);
}

bool UARTProtocol::sendPing() {
    // Ping has no payload
    return sendFrame(UART_MSG_PING, nullptr, 0);
}

bool UARTProtocol::sendPong() {
    // Pong has no payload
    return sendFrame(UART_MSG_PONG, nullptr, 0);
}

bool UARTProtocol::sendFrame(UARTMsgType type, const uint8_t* payload, uint8_t payloadLength) {
    if (!serial) return false;
    if (payloadLength > UART_PROTOCOL_MAX_PAYLOAD) return false;
    
    // Calculate CRC
    uint8_t crc = calculateCRC(payloadLength, (uint8_t)type, payload, payloadLength);
    
    // Send frame: SYNC + LENGTH + TYPE + PAYLOAD + CRC
    serial->write(UART_PROTOCOL_SYNC_BYTE);
    serial->write(payloadLength);
    serial->write((uint8_t)type);
    if (payloadLength > 0 && payload) {
        serial->write(payload, payloadLength);
    }
    serial->write(crc);
    
    packetsSent++;
    return true;
}

uint8_t UARTProtocol::calculateCRC(uint8_t length, uint8_t type, const uint8_t* payload, uint8_t payloadLength) {
    uint8_t data[UART_PROTOCOL_MAX_PAYLOAD + 2];
    data[0] = length;
    data[1] = type;
    if (payloadLength > 0 && payload) {
        memcpy(data + 2, payload, payloadLength);
    }
    return crc8.calc(data, payloadLength + 2);
}
