# RP2040 BLE TX/RX for FPV Drone

A Bluetooth Low Energy (BLE) based remote control system for FPV drones using Raspberry Pi Pico W (RP2040) boards. The transmitter (TX) reads analog sticks and switches, and the receiver (RX) outputs CRSF protocol signals compatible with Betaflight/ELRS flight controllers.

## Hardware Requirements

### TX Side (Remote Control)
- Raspberry Pi Pico W
- SH1106G OLED display (128x64, I2C)
- ADS1115 16-bit ADC module (I2C)
- 2x 2-axis analog joysticks (4 analog inputs)
- 4x toggle switches
- 2x navigation buttons

**Wiring:**
| Component | Pin |
|-----------|-----|
| I2C SDA | GP4 |
| I2C SCL | GP5 |
| OLED Address | 0x3C |
| ADS1115 Address | 0x48 |
| Enter Button | GP18 (active high) |
| Back Button | GP13 (active high) |
| Toggle Switch 1 | GP28 (active low) |
| Toggle Switch 2 | GP21 (active low) |
| Toggle Switch 3 | GP3 (active low) |
| Toggle Switch 4 | GP9 (active low) |

**ADS1115 Channel Mapping:**
| ADS Channel | Function | CRSF Channel |
|-------------|----------|--------------|
| 0 | Aileron (A) | ch0 |
| 1 | Elevator (E) | ch1 |
| 2 | Rudder (R) | ch2 |
| 3 | Throttle (T) | ch3 |

### RX Side (Flight Controller Receiver)
- Raspberry Pi Pico W
- CRSF output: TX → GP8 (connect to flight controller CRSF RX)

## Software Setup

1. Install PlatformIO
2. Clone this repository
3. Build and upload:
   - For TX: `pio run -e tx-ble -t upload`
   - For RX: `pio run -e rx-ble -t upload`

## Pairing TX and RX

The TX and RX use a shared 16-byte encryption key for secure communication. To pair:

1. **Put RX in pairing mode:** Hold the BOOTSEL button on the RX Pico W for 5 seconds. The LED will blink rapidly when pairing mode is active (60 second timeout).

2. **Initiate pairing from TX:** 
   - If already connected: Hold the Back button (GP13) for 3 seconds
   - If not connected: Hold Back for 3 seconds to enter pairing scan mode, then select the RX device

3. The TX generates a new random pairing key and sends it to the RX. Both devices save the key to EEPROM.

4. After successful pairing, the devices will automatically reconnect on subsequent power-ups.

## Channel Mapping

| Channel | Source | Description |
|---------|--------|-------------|
| 0 | ADS1115 Ch0 | Aileron (1000-2000) |
| 1 | ADS1115 Ch1 | Elevator (1000-2000) |
| 2 | ADS1115 Ch2 | Rudder (1000-2000) |
| 3 | ADS1115 Ch3 | Throttle (1000-2000) |
| 4 | Toggle SW1 | Aux1 (1000/2000) |
| 5 | Toggle SW2 | Aux2 (1000/2000) |
| 6 | Toggle SW3 | Aux3 (1000/2000) |
| 7 | Toggle SW4 | Aux4 (1000/2000) |

## BLE Protocol

The system uses Bluetooth Low Energy with a custom GATT service:

| UUID | Name | Properties | Description |
|------|------|------------|-------------|
| 0xFF00 | FPV Service | - | Main service UUID |
| 0xFF01 | TX Char | Write | Channel data (encrypted) |
| 0xFF02 | RX Char | Notify | Telemetry (future) |
| 0xFF03 | Pair Char | Write | Pairing key exchange |

**Frame Format:**
- Encrypted 8-channel data with sequence number
- HMAC authentication for security
- 50Hz update rate (20ms intervals)

## Usage

1. Power on the RX side first - it will start BLE advertising as "FPV-RX"
2. Power on the TX side:
   - If previously paired, it will automatically connect to the bonded RX
   - If not paired, enter the menu to scan and select a device
3. The TX OLED displays connection status and channel values
4. The RX outputs CRSF signals on GP8 for your flight controller

### TX Menu Navigation

- **Enter menu (when connected):** Hold both Enter + Back buttons for 500ms
- **Scan for devices:** Press Enter in main menu
- **Navigate device list:** Use joystick up/down
- **Select device:** Press Enter
- **Back:** Press Back button
- **Pairing mode:** Hold Back for 3 seconds

## Troubleshooting

- **TX won't connect:** Ensure RX is powered and advertising. Check if previously bonded to a different RX.
- **Pairing fails:** Make sure RX is in pairing mode (LED blinking rapidly). Hold BOOTSEL for 5 seconds.
- **No CRSF output:** Verify RX is connected to TX and receiving data (check Serial monitor at 115200 baud)
- **Sticks not responding:** Check ADS1115 I2C wiring and address. The TX logs ADC values to Serial.
- **OLED not working:** Verify I2C wiring (SDA=GP4, SCL=GP5) and address (0x3C for SH1106G)

## Technical Details

### BLE Stack
- Uses BTstack library via Arduino-Pico core
- TX: BLE Central (client) with low-level GATT client API
- RX: BLE Peripheral (server) with BTstackLib GATT server

### Security
- 16-byte shared pairing key stored in EEPROM
- Sequence number to prevent replay attacks
- HMAC authentication on each frame

### CRSF Output
- Standard CRSF protocol at 420000 baud
- 16 channels (8 active, 8 centered)
- 50Hz frame rate
- Compatible with Betaflight, ELRS, and other CRSF receivers

### ADC Configuration
- ADS1115 at 250 SPS (samples per second)
- Gain = 1 (±4.096V range)
- 16-bit resolution
- I2C at 400kHz after initialization

## Dependencies

- [arduino-pico](https://github.com/earlephilhower/arduino-pico) core with Bluetooth enabled
- [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110X) - OLED display driver
- [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) - Graphics library
- [Adafruit ADS1X15](https://github.com/adafruit/Adafruit_ADS1X15) - ADC driver

