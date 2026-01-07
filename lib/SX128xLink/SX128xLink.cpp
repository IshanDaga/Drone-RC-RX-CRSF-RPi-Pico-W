// SX128x LoRa transport helper using RadioLib.
#include "SX128xLink.h"
#include <SPI.h>

volatile bool SX128xLink::packetReceived = false;

SX128xLink::SX128xLink()
    : ready(false),
      module(nullptr),
      radio(nullptr) {}

void SX128xLink::dio1ISR() {
    packetReceived = true;
}

bool SX128xLink::begin() {
    if (ready) {
        return true;
    }

    Serial.print("[SX128x] Initializing with pins: CS=");
    Serial.print(SX128X_SPI_CS);
    Serial.print(", DIO1=");
    Serial.print(SX128X_SPI_DIO1);
    Serial.print(", RST=");
    Serial.print(SX128X_SPI_RST);
    Serial.print(", BUSY=");
    Serial.print(SX128X_SPI_BUSY);
    Serial.print(", RXEN=");
    Serial.print(SX128X_RXEN);
    Serial.print(", TXEN=");
    Serial.println(SX128X_TXEN);

    // Configure SPI pins for RP2040 hardware SPI0 (non-standard SPI setup).
    // RadioLib will not initialize SPI automatically when using extended Module constructor.
    Serial.print("[SX128x] Configuring SPI: SCK=");
    Serial.print(SX128X_SPI_SCK);
    Serial.print(", MOSI=");
    Serial.print(SX128X_SPI_MOSI);
    Serial.print(", MISO=");
    Serial.print(SX128X_SPI_MISO);
    Serial.print(", CS=");
    Serial.println(SX128X_SPI_CS);

    SPI.setSCK(SX128X_SPI_SCK);
    SPI.setTX(SX128X_SPI_MOSI);
    SPI.setRX(SX128X_SPI_MISO);
    SPI.setCS(SX128X_SPI_CS);
    SPI.begin();
    Serial.println("[SX128x] SPI.begin() completed");

    // Use extended Module constructor with SPI instance and settings for non-standard SPI setup.
    // This prevents RadioLib from automatically initializing SPI.
    SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
    Serial.println("[SX128x] Creating Module...");
    module = new Module(SX128X_SPI_CS, SX128X_SPI_DIO1, SX128X_SPI_RST, SX128X_SPI_BUSY, SPI, spiSettings);
    if (!module) {
        Serial.println("[SX128x] ERROR: Failed to create Module");
        return false;
    }
    Serial.println("[SX128x] Module created");

    Serial.println("[SX128x] Creating SX1280...");
    radio = new SX1280(module);
    if (!radio) {
        Serial.println("[SX128x] ERROR: Failed to create SX1280");
        return false;
    }
    Serial.println("[SX128x] SX1280 created");

    // Drive external RF switch (Ebyte TX_EN/RX_EN).
    Serial.println("[SX128x] Setting RF switch pins...");
    radio->setRfSwitchPins(SX128X_RXEN, SX128X_TXEN);
    Serial.println("[SX128x] RF switch pins set");

    // DIO1 interrupt is needed for async RX (startReceive/readData).
    // TX must also receive (pairing ACK, sync ACK, telemetry), so enable DIO1 on both sides.
#if defined(RX_SIDE) || defined(TX_SIDE)
    Serial.println("[SX128x] Setting DIO1 interrupt...");
    radio->setDio1Action(SX128xLink::dio1ISR);
    Serial.println("[SX128x] DIO1 interrupt set");
#endif

    Serial.print("[SX128x] Calling beginFLRC: freq=");
    Serial.print(SX128X_FREQ_MHZ);
    Serial.print("MHz, bitrate=");
    Serial.print(SX128X_FLRC_BR_KBPS);
    Serial.print("kbps, coding_rate=1/");
    Serial.print(SX128X_FLRC_CR);
    Serial.print(" (value=");
    Serial.print(SX128X_FLRC_CR);
    Serial.print("), power=");
    Serial.print(SX128X_OUTPUT_POWER_DBM);
    Serial.print("dBm, preamble=");
    Serial.print(SX128X_FLRC_PREAMBLE_BITS);
    Serial.print("bits, shaping=");
    Serial.println(SX128X_FLRC_SHAPING);
    
    int16_t state = radio->beginFLRC(
        SX128X_FREQ_MHZ,
        SX128X_FLRC_BR_KBPS,
        SX128X_FLRC_CR,
        SX128X_OUTPUT_POWER_DBM,
        SX128X_FLRC_PREAMBLE_BITS,
        SX128X_FLRC_SHAPING
    );
    
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("[SX128x] ERROR: beginFLRC failed with code ");
        Serial.print(state);
        Serial.print(" (expected ");
        Serial.print(RADIOLIB_ERR_NONE);
        Serial.println(")");
        Serial.println("[SX128x] Common errors: -2=CHIP_NOT_FOUND, -10=INVALID_CODING_RATE, -12=INVALID_FREQUENCY, -13=INVALID_BIT_RATE");
        Serial.print("[SX128x] Attempted coding rate value: ");
        Serial.println(SX128X_FLRC_CR);
        Serial.println("[SX128x] Valid FLRC coding rates: 2 (1/2), 4 (3/4), 1 (1)");
        return false;
    }
    Serial.println("[SX128x] beginFLRC successful");

    // Continuous RX after init (both RX and TX need to receive).
#if defined(RX_SIDE) || defined(TX_SIDE)
    Serial.println("[SX128x] Starting receive...");
    startReceive();
    Serial.println("[SX128x] Receive started");
#endif
    
    ready = true;
    Serial.println("[SX128x] Initialization complete!");
    return true;
}

void SX128xLink::startReceive() {
    if (!radio) {
        return;
    }
    packetReceived = false;
    radio->startReceive(RADIOLIB_SX128X_RX_TIMEOUT_NONE);
}

bool SX128xLink::send(const uint8_t* data, size_t len) {
    if (!ready || !radio || !data || len == 0) {
        Serial.print("[SX128x] send() check failed: ready=");
        Serial.print(ready);
        Serial.print(", radio=");
        Serial.print(radio ? "OK" : "NULL");
        Serial.print(", data=");
        Serial.print(data ? "OK" : "NULL");
        Serial.print(", len=");
        Serial.println(len);
        return false;
    }

    uint8_t buffer[64];
    if (len > sizeof(buffer)) {
        Serial.print("[SX128x] Packet too long: ");
        Serial.print(len);
        Serial.print(" > ");
        Serial.println(sizeof(buffer));
        return false;
    }
    memcpy(buffer, data, len);

    // Pad to a fixed packet length if configured (improves interoperability for FLRC fixed-length configs).
    size_t txLen = len;
#if defined(SX128X_FIXED_PACKET_LEN) && (SX128X_FIXED_PACKET_LEN > 0)
    if (SX128X_FIXED_PACKET_LEN > sizeof(buffer)) {
        Serial.print("[SX128x] ERROR: SX128X_FIXED_PACKET_LEN too large: ");
        Serial.print(SX128X_FIXED_PACKET_LEN);
        Serial.print(" > ");
        Serial.println(sizeof(buffer));
        return false;
    }
    if (len < (size_t)SX128X_FIXED_PACKET_LEN) {
        memset(buffer + len, 0, (size_t)SX128X_FIXED_PACKET_LEN - len);
        txLen = (size_t)SX128X_FIXED_PACKET_LEN;
    }
#endif

    Serial.print("[SX128x] Calling transmit() with len=");
    Serial.print(txLen);
    if (txLen != len) {
        Serial.print(" (padded from ");
        Serial.print(len);
        Serial.print(")");
    }
    Serial.println();
    int16_t state = radio->transmit(buffer, txLen);
    
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("[SX128x] transmit() failed with code ");
        Serial.print(state);
        Serial.print(" (expected ");
        Serial.print(RADIOLIB_ERR_NONE);
        Serial.println(")");
        Serial.println("[SX128x] Common TX errors: -5=TX_TIMEOUT, -20=WRONG_MODEM");
    } else {
        Serial.println("[SX128x] transmit() successful");
    }
    
    // Return to RX immediately after TX so we can receive ACK/telemetry.
#if defined(RX_SIDE) || defined(TX_SIDE)
    startReceive();
#endif
    return state == RADIOLIB_ERR_NONE;
}

bool SX128xLink::receive(SX128xPacket& packet) {
    if (!ready || !radio || !packetReceived) {
        return false;
    }

    packetReceived = false;
    // Respect reported packet length to avoid stale bytes in buffer.
    size_t len = radio->getPacketLength();
    if (len > sizeof(packet.data)) {
        len = sizeof(packet.data);
    }

    int16_t state = radio->readData(packet.data, len);
    if (state != RADIOLIB_ERR_NONE) {
        startReceive();
        return false;
    }

    packet.length = len;
    packet.rssi = radio->getRSSI();
    packet.snr = radio->getSNR();

    // Continue listening.
    startReceive();
    return true;
}


