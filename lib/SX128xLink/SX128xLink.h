// Lightweight wrapper around RadioLib SX128x for shared TX/RX use.
#ifndef SX128X_LINK_H
#define SX128X_LINK_H

#include <Arduino.h>
#include <RadioLib.h>

#ifndef SX128X_SPI_SCK
#define SX128X_SPI_SCK 18
#endif

#ifndef SX128X_SPI_MOSI
#define SX128X_SPI_MOSI 19
#endif

#ifndef SX128X_SPI_MISO
#define SX128X_SPI_MISO 16
#endif

#ifndef SX128X_SPI_CS
#define SX128X_SPI_CS 17
#endif

#ifndef SX128X_SPI_BUSY
#define SX128X_SPI_BUSY 20
#endif

#ifndef SX128X_SPI_DIO1
#define SX128X_SPI_DIO1 21
#endif

#ifndef SX128X_SPI_RST
#define SX128X_SPI_RST 22
#endif

#ifndef SX128X_FREQ_MHZ
#define SX128X_FREQ_MHZ 2420.0f
#endif

#ifndef SX128X_BW_KHZ
#define SX128X_BW_KHZ 812.5f
#endif

#ifndef SX128X_SF
#define SX128X_SF 7
#endif

#ifndef SX128X_CR
#define SX128X_CR 5  // 4/5
#endif

#ifndef SX128X_OUTPUT_POWER_DBM
#define SX128X_OUTPUT_POWER_DBM 10
#endif

#ifndef SX128X_RXEN
#define SX128X_RXEN 14
#endif

#ifndef SX128X_TXEN
#define SX128X_TXEN 15
#endif

#ifndef SX128X_FLRC_BR_KBPS
#define SX128X_FLRC_BR_KBPS 1300  // prioritize lowest latency (1.3 Mbps)
#endif

#ifndef SX128X_FLRC_CR
// FLRC coding rate: denominator value (2 = 1/2, 4 = 3/4, 1 = 1)
// Using 2 (1/2) as default - most compatible
#define SX128X_FLRC_CR 2
#endif

#ifndef SX128X_FLRC_PREAMBLE_BITS
#define SX128X_FLRC_PREAMBLE_BITS 16
#endif

#ifndef SX128X_FLRC_SHAPING
#define SX128X_FLRC_SHAPING RADIOLIB_SHAPING_0_5
#endif

// Some SX1280 FLRC configurations behave like fixed-length packets (especially after TX).
// To keep TX/RX interoperable across mixed-length message types, pad all packets to a
// single fixed length.
// Set to 0 to disable padding (variable-length).
#ifndef SX128X_FIXED_PACKET_LEN
#define SX128X_FIXED_PACKET_LEN 33
#endif

struct SX128xPacket {
    uint8_t data[64];
    size_t length;
    int16_t rssi;
    float snr;
};

class SX128xLink {
public:
    SX128xLink();
    bool begin();
    bool send(const uint8_t* data, size_t len);
    bool receive(SX128xPacket& packet);
    void startReceive();
    bool isReady() const { return ready; }

private:
    static void dio1ISR();
    static volatile bool packetReceived;
    bool ready;
    Module* module;
    SX1280* radio;
};

#endif // SX128X_LINK_H


