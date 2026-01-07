// RC-CRSF: Remote control that outputs CRSF over serial to TX side.
// Handles display, battery, analog inputs, UI - everything except radio comms.
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

#include "UARTProtocol.h"

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define I2C_ADDRESS 0x3C

// ADS1115 Configuration
#define ADS1115_ADDRESS 0x48  // Default I2C address (can be 0x48-0x4B)

// Pin definitions
// I2C - ADC
#define I2C_SDA 4
#define I2C_SCL 5

// CRSF Serial pins (outputting CRSF to TX side)
#define CRSF_TX_PIN 8
#define CRSF_RX_PIN 9

// ADS1115 channel assignments (0-3) → CRSF mapping: A,E,R,T
// ADS ch0 → channels[0] → CRSF ch0 = Aileron  (A)
// ADS ch1 → channels[1] → CRSF ch1 = Elevator (E)
// ADS ch2 → channels[2] → CRSF ch2 = Rudder   (R)
// ADS ch3 → channels[3] → CRSF ch3 = Throttle (T)
#define AILERON_CHANNEL  0  // Stick1 X
#define ELEVATOR_CHANNEL 1  // Stick1 Y
#define RUDDER_CHANNEL   2  // Stick2 X
#define THROTTLE_CHANNEL 3  // Stick2 Y

// Toggle switches (dual-pin reading)
// Logic: IN1 high = 1000, IN2 high = 2000, both low = 1500
// T1: GPIO1 (T1IN1), GPIO2 (T1IN2)
// T2: GPIO3 (T2IN1), GPIO6 (T2IN2)
// T3: GPIO7 (T3IN1), GPIO10 (T3IN2)
// T4: GPIO11 (T4IN1), GPIO12 (T4IN2)
#define TOGGLE_SWITCH1_IN1_PIN 1   // T1IN1
#define TOGGLE_SWITCH1_IN2_PIN 2   // T1IN2
#define TOGGLE_SWITCH2_IN1_PIN 3   // T2IN1
#define TOGGLE_SWITCH2_IN2_PIN 6   // T2IN2
#define TOGGLE_SWITCH3_IN1_PIN 7   // T3IN1
#define TOGGLE_SWITCH3_IN2_PIN 10  // T3IN2
#define TOGGLE_SWITCH4_IN1_PIN 11  // T4IN1
#define TOGGLE_SWITCH4_IN2_PIN 12  // T4IN2

// Menu navigation buttons (active high)
#define ENTER_BUTTON_PIN 13  // CN3 [push button 1]
#define BACK_BUTTON_PIN 14   // CN4 [push button 2]

// SPI Display pins (reserved for future use if needed)
#define SPI_SCK_PIN 18
#define SPI_TX_PIN 19
#define SPI_RX_PIN 20
#define SPI_CSN_PIN 21

// Buzzer (for boot jingle)
#define BUZZER_PIN 24

// BQ25620 charger / fuel (ADC) over I2C
#define BQ25620_I2C_ADDR       0x6A  // Scan showed device at 0x6A
#define BQ25620_REG_ADC_CONTROL 0x26
#define BQ25620_REG_ADC_FUNC_DIS0 0x27
#define BQ25620_REG_VBAT_ADC    0x30
#define BQ25620_ADC_ENABLE_BIT  0x80
#define BQ25620_ADC_STEP_V      0.0010f   // Empirically closer for VBAT (was 1.99mV)
#define TX_BATT_EMPTY_V         3.2f
#define TX_BATT_FULL_V          4.2f

// EEPROM addresses
#define EEPROM_SIZE 512
#define CAL_MAGIC_ADDR 150
#define CAL_DATA_ADDR 152
#define CAL_MAGIC_VALUE 0xC5

// Timing
static unsigned long lastChannelSend = 0;
static const unsigned long CHANNEL_INTERVAL = 4;  // 250Hz (higher than 200Hz for safety margin)
static unsigned long lastDisplayUpdate = 0;
static const unsigned long DISPLAY_INTERVAL = 200; // ms
static unsigned long lastPingSent = 0;
static const unsigned long PING_INTERVAL = 1000;  // 1Hz ping for device check
static unsigned long lastPongReceived = 0;
static const unsigned long PONG_TIMEOUT_MS = 2000;  // TX considered disconnected if no pong for 2s

// Application state
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;  // ADS1115 ADC on I2C bus
UARTProtocol uartProto(&Serial2);

// Telemetry data from TX
static TelemetryData telemetry = {0};
static StatusData status = {0};
static bool telemetryValid = false;
static bool statusValid = false;
static unsigned long lastTelemetryUpdate = 0;
static const unsigned long TELEMETRY_TIMEOUT_MS = 2000;

// Button state for command handling
static bool enterButtonPressed = false;
static bool backButtonPressed = false;
static unsigned long lastEnterPress = 0;
static unsigned long lastBackPress = 0;
static const unsigned long BUTTON_DEBOUNCE_MS = 200;

// Device status flags
bool ads_ok = false;
bool display_ok = false;
bool tx_connected = false;  // TX board connection status

// Channel values (mapped to CRSF range 1000-2000)
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Raw input snapshots (for logging/calibration)
int16_t raw_adc[4] = {0};
bool raw_toggles[4] = {false};

// Calibration data for sticks
typedef struct {
    uint16_t min[4];
    uint16_t max[4];
    uint16_t center[4];
} calib_data_t;

static calib_data_t calib = {
    {2917, 2917, 2917, 2917},
    {23420, 23420, 23420, 23420},
    {13199, 13199, 13199, 13199}
};
static bool calib_loaded = false;

// TX local battery (BQ25620)
static float tx_batt_voltage = 0.0f;
static uint8_t tx_batt_percent = 0;
static uint8_t tx_batt_bars = 0;
static bool tx_batt_valid = false;
static unsigned long last_tx_batt_read = 0;
static const unsigned long TX_BATT_READ_INTERVAL = 5000; // ms
static uint16_t tx_batt_raw_last = 0;

// Forward declarations
static void load_calibration(void);
static bool init_ads1115(void);
static void readInputs();
static void updateDisplay();
static void sendChannels();
static void receiveUARTMessages();
static void handleButtons();
static uint8_t voltage_to_percent(float v);
static uint8_t percent_to_bars(uint8_t pct);
static bool bq25620_begin(void);
static bool bq25620_read_voltage(float &voltage_out, uint16_t &raw_out);
static void update_tx_battery(void);
static void play_boot_jingle(void);
static void scan_i2c_bus(void);
static bool init_display(void);

// UART Protocol callbacks
static void onTelemetryReceived(const TelemetryData* data);
static void onStatusReceived(const StatusData* data);
static void onAckReceived(UARTMsgType ackedCmd);
static void onErrorReceived(uint8_t errorCode);
static void onPongReceived();

// Device check functions
static bool check_tx_connection(void);
static void ping_tx_device(void);

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
    delay(200);

    Serial.println("=== RC-CRSF Controller ===");
    Serial.println("Display, battery, analog inputs -> CRSF serial output");

    // Initialize I2C for OLED and ADS1115
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    
    // Start with standard I2C speed (100kHz) for compatibility
    Wire.setClock(100000);  // 100kHz standard mode
    Serial.println("I2C bus initialized at 100kHz");
    Serial.print("I2C pins: SDA=GP");
    Serial.print(I2C_SDA);
    Serial.print(", SCL=GP");
    Serial.println(I2C_SCL);

    // Scan I2C bus to see what devices are present
    scan_i2c_bus();

    // Initialize display with error checking
    display_ok = init_display();
    if (!display_ok) {
        Serial.println("ERROR: Display initialization failed!");
        Serial.println("Check wiring: SDA=GP4, SCL=GP5, VCC=3.3V, GND=GND");
    }

    // Initialize ADS1115
    ads_ok = init_ads1115();

    // Initialize BQ25620 ADC for TX battery monitoring
    if (bq25620_begin()) {
        Serial.println("BQ25620 ADC enabled");
    } else {
        Serial.println("BQ25620 init failed");
    }

    // Initialize buttons and toggles
    pinMode(ENTER_BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(BACK_BUTTON_PIN, INPUT_PULLDOWN);
    // Toggle switches - both IN1 and IN2 pins configured as INPUT_PULLUP
    pinMode(TOGGLE_SWITCH1_IN1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH1_IN2_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH2_IN1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH2_IN2_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH3_IN1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH3_IN2_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH4_IN1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH4_IN2_PIN, INPUT_PULLUP);

    // Initialize buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Load stick calibration (if present)
    load_calibration();

    // Initialize UART protocol (outputting to TX side)
    Serial.println("Initializing UART protocol...");
    Serial2.setTX(CRSF_TX_PIN);
    Serial2.setRX(CRSF_RX_PIN);
    uartProto.begin(UART_PROTOCOL_BAUDRATE);
    uartProto.setOnTelemetry(onTelemetryReceived);
    uartProto.setOnStatus(onStatusReceived);
    uartProto.setOnAck(onAckReceived);
    uartProto.setOnError(onErrorReceived);
    uartProto.setOnPong(onPongReceived);
    Serial.print("UART Protocol on Serial2: TX=GP");
    Serial.print(CRSF_TX_PIN);
    Serial.print(", RX=GP");
    Serial.println(CRSF_RX_PIN);
    
    // Check TX connection - send initial ping after a short delay
    Serial.println("Checking TX board connection...");
    delay(100);  // Give UART time to stabilize
    lastPingSent = 0;  // Reset to allow immediate ping
    uartProto.sendPing();  // Send initial ping immediately
    lastPingSent = millis();
    Serial.println("[TX] Initial ping sent");
    check_tx_connection();  // Check initial status

    if (display_ok) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("RC-CRSF ready");
        display.display();
    }

    // Play boot jingle
    play_boot_jingle();
    
    // Increase I2C speed after initialization if devices are working
    if (display_ok || ads_ok) {
        Wire.setClock(400000);  // 400kHz fast mode
        Serial.println("I2C bus speed increased to 400kHz");
    }
}

// ============================================================
// Loop
// ============================================================
void loop() {
    readInputs();
    sendChannels();
    receiveUARTMessages();
    ping_tx_device();  // Periodic device check
    check_tx_connection();  // Update connection status
    handleButtons();
    update_tx_battery();
    updateDisplay();
}

// ============================================================
// Input handling
// ============================================================
void readInputs() {
    // Read analog sticks from ADS1115 (16-bit ADC: 0-32767)
    int16_t stick1_x = 16384;  // Center value (32767/2)
    int16_t stick1_y = 16384;
    int16_t stick2_x = 16384;
    int16_t stick2_y = 16384;

    if (ads_ok) {
        stick1_x = ads.readADC_SingleEnded(AILERON_CHANNEL);   // Aileron
        stick1_y = ads.readADC_SingleEnded(ELEVATOR_CHANNEL);  // Elevator
        stick2_x = ads.readADC_SingleEnded(RUDDER_CHANNEL);    // Rudder
        stick2_y = ads.readADC_SingleEnded(THROTTLE_CHANNEL);  // Throttle
    }

    raw_adc[0] = stick1_x;
    raw_adc[1] = stick1_y;
    raw_adc[2] = stick2_x;
    raw_adc[3] = stick2_y;

    // Apply deadzone on ADC (5%) before mapping
    auto applyDeadzone = [](int16_t val, uint16_t vmin, uint16_t vmax, float frac) -> int16_t {
        if (vmax <= vmin + 10) return val;
        float center = (vmin + vmax) * 0.5f;
        float halfRange = (vmax - vmin) * 0.5f;
        float dz = halfRange * frac;
        float delta = val - center;
        if (fabsf(delta) < dz) return (int16_t)center;
        float sign = (delta > 0) ? 1.0f : -1.0f;
        float adjusted = sign * ((fabsf(delta) - dz) / (halfRange - dz)) * halfRange;
        return (int16_t)(center + adjusted);
    };
    const float DEADZONE = 0.05f; // 5%
    stick1_x = applyDeadzone(stick1_x, calib.min[0], calib.max[0], DEADZONE);
    stick1_y = applyDeadzone(stick1_y, calib.min[1], calib.max[1], DEADZONE);
    stick2_x = applyDeadzone(stick2_x, calib.min[2], calib.max[2], DEADZONE);
    // Leave throttle without deadzone to preserve full resolution

    // Map to channel values (1000-2000)
    auto mapCal = [](int16_t val, uint16_t vmin, uint16_t vmax) -> uint16_t {
        if (vmax <= vmin + 10) return 1500; // avoid div0
        return (uint16_t)constrain(map(val, vmin, vmax, 1000, 2000), 1000, 2000);
    };
    auto mapCalInverted = [](int16_t val, uint16_t vmin, uint16_t vmax) -> uint16_t {
        if (vmax <= vmin + 10) return 1500; // avoid div0
        return (uint16_t)constrain(map(val, vmin, vmax, 2000, 1000), 1000, 2000);
    };
    channels[0] = mapCalInverted(stick1_x, calib.min[0], calib.max[0]);  // Aileron (invert)
    channels[1] = mapCal(stick1_y, calib.min[1], calib.max[1]);          // Elevator
    channels[2] = mapCal(stick2_x, calib.min[2], calib.max[2]);          // Rudder
    channels[3] = mapCal(stick2_y, calib.min[3], calib.max[3]);          // Throttle

    // Apply EdgeTX-style dual-rate/expo
    auto applyCurve = [](uint16_t us, float rate, float expo) -> uint16_t {
        float x = (int(us) - 1500) / 500.0f;                  // -1..1
        float x_expo = (1.0f - expo) * x + expo * x * x * x;  // soften center
        float scaled = x_expo * rate;                         // cap max throw
        int out = int(1500 + scaled * 500);
        return (uint16_t)constrain(out, 1000, 2000);
    };
    const float RATE = 0.7f;
    const float EXPO = 0.3f;
    channels[0] = applyCurve(channels[0], RATE, EXPO);
    channels[1] = applyCurve(channels[1], RATE, EXPO);
    channels[3] = applyCurve(channels[3], RATE, EXPO);
    channels[0] = (uint16_t)constrain(channels[0], 1300, 1700);
    channels[1] = (uint16_t)constrain(channels[1], 1300, 1700);

    // Read toggle switches (dual-pin reading)
    // Logic: IN1 high = 1000, IN2 high = 2000, both low = 1500
    auto readToggle = [](uint8_t in1_pin, uint8_t in2_pin) -> uint16_t {
        bool in1_high = digitalRead(in1_pin) == HIGH;
        bool in2_high = digitalRead(in2_pin) == HIGH;
        
        if (in1_high && !in2_high) {
            return 1000;  // IN1 high, IN2 low
        } else if (!in1_high && in2_high) {
            return 2000;  // IN1 low, IN2 high
        } else {
            return 1500;  // Both low (or both high - treat as center)
        }
    };
    
    channels[4] = readToggle(TOGGLE_SWITCH1_IN1_PIN, TOGGLE_SWITCH1_IN2_PIN);
    channels[5] = readToggle(TOGGLE_SWITCH2_IN1_PIN, TOGGLE_SWITCH2_IN2_PIN);
    channels[6] = readToggle(TOGGLE_SWITCH3_IN1_PIN, TOGGLE_SWITCH3_IN2_PIN);
    channels[7] = readToggle(TOGGLE_SWITCH4_IN1_PIN, TOGGLE_SWITCH4_IN2_PIN);
    
    // Store raw toggle states for logging (using IN1 state as primary indicator)
    raw_toggles[0] = digitalRead(TOGGLE_SWITCH1_IN1_PIN) == HIGH;
    raw_toggles[1] = digitalRead(TOGGLE_SWITCH2_IN1_PIN) == HIGH;
    raw_toggles[2] = digitalRead(TOGGLE_SWITCH3_IN1_PIN) == HIGH;
    raw_toggles[3] = digitalRead(TOGGLE_SWITCH4_IN1_PIN) == HIGH;
}

// ============================================================
// UART Protocol - Channel Output
// ============================================================
void sendChannels() {
    if (millis() - lastChannelSend < CHANNEL_INTERVAL) return;
    lastChannelSend = millis();

    uartProto.sendChannels(channels);
}

// ============================================================
// UART Protocol - Message Reception
// ============================================================
void receiveUARTMessages() {
    uartProto.loop();
    
    // Check telemetry timeout
    if (telemetryValid && (millis() - lastTelemetryUpdate) > TELEMETRY_TIMEOUT_MS) {
        telemetryValid = false;
    }
}

// ============================================================
// UART Protocol - Callbacks
// ============================================================
void onTelemetryReceived(const TelemetryData* data) {
    if (data) {
        memcpy(&telemetry, data, sizeof(TelemetryData));
        telemetryValid = true;
        lastTelemetryUpdate = millis();
    }
}

void onStatusReceived(const StatusData* data) {
    if (data) {
        memcpy(&status, data, sizeof(StatusData));
        statusValid = true;
    }
}

void onAckReceived(UARTMsgType ackedCmd) {
    Serial.print("[UART] ACK received for command: 0x");
    Serial.println((uint8_t)ackedCmd, HEX);
}

void onErrorReceived(uint8_t errorCode) {
    Serial.print("[UART] Error received: 0x");
    Serial.println(errorCode, HEX);
}

void onPongReceived() {
    lastPongReceived = millis();
    if (!tx_connected) {
        tx_connected = true;
        Serial.println("[TX] Device connected!");
    }
}

// ============================================================
// Button Handling for Commands
// ============================================================
void handleButtons() {
    bool enterPressed = digitalRead(ENTER_BUTTON_PIN) == HIGH;
    bool backPressed = digitalRead(BACK_BUTTON_PIN) == HIGH;
    
    // ENTER button - PAIR command
    if (enterPressed && !enterButtonPressed && (millis() - lastEnterPress) > BUTTON_DEBOUNCE_MS) {
        enterButtonPressed = true;
        lastEnterPress = millis();
        uartProto.sendCommand(UART_MSG_CMD_PAIR);
        Serial.println("[RC] Sending PAIR command");
    } else if (!enterPressed) {
        enterButtonPressed = false;
    }
    
    // BACK button - STATUS_REQ command
    if (backPressed && !backButtonPressed && (millis() - lastBackPress) > BUTTON_DEBOUNCE_MS) {
        backButtonPressed = true;
        lastBackPress = millis();
        uartProto.sendCommand(UART_MSG_CMD_STATUS_REQ);
        Serial.println("[RC] Sending STATUS_REQ command");
    } else if (!backPressed) {
        backButtonPressed = false;
    }
}

// ============================================================
// Display
// ============================================================
void updateDisplay() {
    if (millis() - lastDisplayUpdate < DISPLAY_INTERVAL) return;
    lastDisplayUpdate = millis();

    // Only update display if it was successfully initialized
    if (!display_ok) return;

    display.clearDisplay();
    display.setCursor(0, 0);
    
    // Line 1: RC-CRSF, TX battery, TX connection, radio status
    display.print("RC-CRSF  ");
    display.print(tx_batt_percent);
    display.print("%  ");
    // TX connection status
    if (tx_connected) {
        display.print("TX:");
        if (statusValid) {
            const char* stateNames[] = {"DISC", "PAIR", "CONN", "OK", "LOST"};
            if (status.connectionState < 5) {
                display.print(stateNames[status.connectionState]);
            }
        } else {
            display.print("OK");
        }
    } else {
        display.print("TX:---");
    }
    display.println();

    // Line 2: RSSI and SNR
    if (telemetryValid) {
        display.print("RSSI:");
        display.print(telemetry.rssi);
        display.print(" SNR:");
        display.print(telemetry.snr, 1);
    } else {
        display.print("No telemetry");
    }
    display.println();

    // Line 3: Channels 0/1
    display.print("Ch0/1:");
    display.print(channels[0]);
    display.print("/");
    display.println(channels[1]);
    
    // Line 4: RX Battery
    if (telemetryValid) {
        display.print("RX: ");
        display.print(telemetry.rxBattMv / 1000.0f, 1);
        display.print("V ");
        display.print(telemetry.rxBattPct);
        display.println("%");
    } else {
        display.print("Ch2/3:");
        display.print(channels[2]);
        display.print("/");
        display.println(channels[3]);
    }

    display.display();
}

// ============================================================
// TX Device Connection Check
// ============================================================
void ping_tx_device(void) {
    if (millis() - lastPingSent < PING_INTERVAL) return;
    lastPingSent = millis();
    
    static unsigned long lastPingLog = 0;
    if (millis() - lastPingLog > 5000) {  // Log every 5 seconds to avoid spam
        Serial.print("[TX] Sending ping (TX connected: ");
        Serial.print(tx_connected ? "YES" : "NO");
        Serial.println(")...");
        lastPingLog = millis();
    }
    
    uartProto.sendPing();
}

bool check_tx_connection(void) {
    // Check if we've received a pong recently
    unsigned long timeSincePong = (lastPongReceived > 0) ? (millis() - lastPongReceived) : PONG_TIMEOUT_MS + 1;
    
    bool wasConnected = tx_connected;
    tx_connected = (timeSincePong < PONG_TIMEOUT_MS);
    
    if (wasConnected != tx_connected) {
        if (tx_connected) {
            Serial.println("[TX] Device connected");
        } else {
            Serial.println("[TX] Device disconnected or not responding");
            Serial.print("[TX] Last pong received: ");
            if (lastPongReceived > 0) {
                Serial.print(timeSincePong);
                Serial.println("ms ago");
            } else {
                Serial.println("never");
            }
        }
    }
    
    // Log initial status check
    static bool initialCheckDone = false;
    if (!initialCheckDone) {
        initialCheckDone = true;
        if (tx_connected) {
            Serial.println("[TX] Initial check: Device connected");
        } else {
            Serial.println("[TX] Initial check: Device not responding (waiting for pong...)");
        }
    }
    
    return tx_connected;
}

// ============================================================
// Helpers
// ============================================================
static void load_calibration(void) {
    EEPROM.begin(EEPROM_SIZE);
    if (EEPROM.read(CAL_MAGIC_ADDR) != CAL_MAGIC_VALUE) {
        Serial.println("Calibration not found, using defaults");
        calib_loaded = false;
        return;
    }
    EEPROM.get(CAL_DATA_ADDR, calib);
    for (int i = 0; i < 4; i++) {
        if (calib.min[i] >= calib.max[i]) {
            calib.min[i] = 2917;
            calib.max[i] = 23420;
        }
        if (calib.center[i] == 0 || calib.center[i] >= calib.max[i]) {
            calib.center[i] = 13199;
        }
    }
    calib_loaded = true;
    Serial.println("Calibration loaded");
}

// ==========================================
// I2C Bus Scanning
// ==========================================
static void scan_i2c_bus(void) {
    Serial.println("Scanning I2C bus...");
    byte error, address;
    int nDevices = 0;
    
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            
            // Identify common devices
            if (address == 0x3C || address == 0x3D) {
                Serial.print(" (OLED/Display - SH1106/SSD1306)");
            } else if (address >= 0x48 && address <= 0x4B) {
                Serial.print(" (ADS1115 ADC)");
            } else if (address == 0x6A) {
                Serial.print(" (BQ25620 Charger)");
            }
            Serial.println(" !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    
    if (nDevices == 0) {
        Serial.println("No I2C devices found!");
        Serial.println("Check wiring: SDA=GP4, SCL=GP5, VCC=3.3V, GND=GND");
    } else {
        Serial.print("Found ");
        Serial.print(nDevices);
        Serial.println(" device(s)");
    }
}

// ==========================================
// Display Initialization
// ==========================================
static bool init_display(void) {
    Serial.print("Attempting to initialize display at address 0x");
    Serial.println(I2C_ADDRESS, HEX);
    
    // Wait a bit for display to power up
    delay(250);
    
    // Try default address first
    if (!display.begin(I2C_ADDRESS, false)) {  // false = no splash screen
        Serial.println("Failed to initialize display at 0x3C");
        Serial.println("Trying alternative address 0x3D...");
        
        // SH1106 can sometimes be at 0x3D
        if (!display.begin(0x3D, false)) {
            Serial.println("Display not found at 0x3C or 0x3D");
            Serial.println("Check wiring: SDA=GP4, SCL=GP5, VCC=3.3V, GND=GND");
            return false;
        } else {
            Serial.println("Display found at address 0x3D");
        }
    } else {
        Serial.print("Display initialized successfully at address 0x");
        Serial.println(I2C_ADDRESS, HEX);
    }
    
    // Configure display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("RC-CRSF init...");
    display.display();
    
    return true;
}

static bool init_ads1115(void) {
    Serial.print("Attempting to initialize ADS1115 at address 0x");
    Serial.println(ADS1115_ADDRESS, HEX);

    if (!ads.begin(ADS1115_ADDRESS, &Wire)) {
        Serial.println("Failed to initialize ADS1115");
        Serial.println("Trying alternative addresses...");

        uint8_t alt_addresses[] = {0x49, 0x4A, 0x4B};
        bool found = false;
        for (uint8_t addr : alt_addresses) {
            Serial.print("Trying 0x");
            Serial.println(addr, HEX);
            if (ads.begin(addr, &Wire)) {
                Serial.print("ADS1115 found at address 0x");
                Serial.println(addr, HEX);
                found = true;
                break;
            }
        }

        if (!found) {
            Serial.println("ADS1115 not found at any address");
            Serial.println("Check wiring: SDA=GP4, SCL=GP5, VCC=3.3V, GND=GND");
            Serial.println("ADDR pin: GND=0x48, VDD=0x49, SDA=0x4A, SCL=0x4B");
            return false;
        }
    } else {
        Serial.print("ADS1115 initialized successfully at address 0x");
        Serial.println(ADS1115_ADDRESS, HEX);
    }

    // Configure ADS1115
    ads.setGain(GAIN_ONE); // ±4.096V range
    ads.setDataRate(RATE_ADS1115_250SPS);
    Serial.println("ADS1115 configured: Gain=1, DataRate=250 SPS");

    // Test read to verify ADC is working
    int16_t test = ads.readADC_SingleEnded(0);
    Serial.print("ADS1115 test read CH0: ");
    Serial.println(test);

    return true;
}

static uint8_t voltage_to_percent(float v) {
    if (v <= TX_BATT_EMPTY_V) return 0;
    if (v >= TX_BATT_FULL_V) return 100;
    return (uint8_t)constrain(((v - TX_BATT_EMPTY_V) / (TX_BATT_FULL_V - TX_BATT_EMPTY_V)) * 100.0f, 0.0f, 100.0f);
}

static uint8_t percent_to_bars(uint8_t pct) {
    if (pct >= 90) return 5;
    if (pct >= 70) return 4;
    if (pct >= 50) return 3;
    if (pct >= 30) return 2;
    if (pct >= 10) return 1;
    return 0;
}

// ==========================================
// BQ25620 Helpers (TX battery)
// ==========================================
static bool bq25620_read8(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(BQ25620_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(BQ25620_I2C_ADDR, (uint8_t)1) != 1) return false;
    val = Wire.read();
    return true;
}

static bool bq25620_write8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(BQ25620_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool bq25620_begin(void) {
    uint8_t adc_ctrl = 0;
    if (!bq25620_read8(BQ25620_REG_ADC_CONTROL, adc_ctrl)) {
        return false;
    }
    // Enable ADC_EN (bit7)
    adc_ctrl |= BQ25620_ADC_ENABLE_BIT;
    if (!bq25620_write8(BQ25620_REG_ADC_CONTROL, adc_ctrl)) {
        return false;
    }
    // Touch function-disable register to confirm comms (keep default)
    uint8_t dummy;
    bq25620_read8(BQ25620_REG_ADC_FUNC_DIS0, dummy);
    return true;
}

static bool bq25620_read_voltage(float &voltage_out, uint16_t &raw_out) {
    // VBAT ADC is 12-bit in 2 bytes, MSB first
    Wire.beginTransmission(BQ25620_I2C_ADDR);
    Wire.write(BQ25620_REG_VBAT_ADC);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(BQ25620_I2C_ADDR, (uint8_t)2) != 2) return false;
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t raw = ((uint16_t)msb << 4) | (lsb >> 4);
    if (raw == 0 || raw == 0x0FFF) {
        return false;
    }
    raw_out = raw;
    voltage_out = raw * BQ25620_ADC_STEP_V; // volts
    return true;
}

static void update_tx_battery(void) {
    if (millis() - last_tx_batt_read < TX_BATT_READ_INTERVAL) return;
    last_tx_batt_read = millis();

    float v = 0.0f;
    uint16_t raw = 0;
    if (bq25620_read_voltage(v, raw)) {
        tx_batt_voltage = v;
        tx_batt_percent = voltage_to_percent(tx_batt_voltage);
        tx_batt_bars = percent_to_bars(tx_batt_percent);
        tx_batt_valid = true;
        tx_batt_raw_last = raw;
        Serial.print("[BQ25620] raw:");
        Serial.print(raw);
        Serial.print(" V:");
        Serial.print(tx_batt_voltage, 3);
        Serial.print(" Pct:");
        Serial.print(tx_batt_percent);
        Serial.println("%");
    } else {
        static unsigned long lastFailLog = 0;
        if (millis() - lastFailLog > 10000) {
            Serial.println("BQ25620 read failed");
            lastFailLog = millis();
        }
    }
}

// ==========================================
// Buzzer - Boot Jingle
// ==========================================
static void play_boot_jingle(void) {
    // buzz the led thrice 
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(500);
        digitalWrite(BUZZER_PIN, LOW);
        delay(500);
    }
    Serial.println("Boot jingle played");
}
