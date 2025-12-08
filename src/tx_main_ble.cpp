// TX Side - BLE Central using low-level BTstack API
// This bypasses BTstackLib which has known issues

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_ADS1X15.h>
#include "Protocol.h"
#include "Security.h"

// Use BTstackLib for initialization, but with our own low-level callbacks
#include <BTstackLib.h>

// Low-level BTstack includes for custom GATT handling
extern "C" {
#include <btstack.h>
#include <ble/gatt_client.h>
#include <ble/sm.h>
}

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define I2C_ADDRESS 0x3C

// ADS1115 Configuration
#define ADS1115_ADDRESS 0x48  // Default I2C address (can be 0x48-0x4B)

// Pin definitions
#define I2C_SDA 4
#define I2C_SCL 5

// ADS1115 channel assignments (0-3) → CRSF mapping: A,E,R,T
// ADS ch0 → channels[0] → CRSF ch0 = Aileron  (A)
// ADS ch1 → channels[1] → CRSF ch1 = Elevator (E)
// ADS ch2 → channels[2] → CRSF ch2 = Rudder   (R)
// ADS ch3 → channels[3] → CRSF ch3 = Throttle (T)
#define AILERON_CHANNEL  0  // Stick1 X
#define ELEVATOR_CHANNEL 1  // Stick1 Y
#define RUDDER_CHANNEL   2  // Stick2 X
#define THROTTLE_CHANNEL 3  // Stick2 Y

// Menu navigation buttons (active high)
#define ENTER_BUTTON_PIN 18
#define BACK_BUTTON_PIN 13

// Toggle switches
#define TOGGLE_SWITCH1_PIN 28
#define TOGGLE_SWITCH2_PIN 21
#define TOGGLE_SWITCH3_PIN 3
#define TOGGLE_SWITCH4_PIN 9

// BQ25620 charger / fuel (ADC) over I2C
#define BQ25620_I2C_ADDR       0x6A  // Scan showed device at 0x6A
#define BQ25620_REG_ADC_CONTROL 0x26
#define BQ25620_REG_ADC_FUNC_DIS0 0x27
#define BQ25620_REG_VBAT_ADC    0x30
#define BQ25620_ADC_ENABLE_BIT  0x80
#define BQ25620_ADC_STEP_V      0.0010f   // Empirically closer for VBAT (was 1.99mV)
#define TX_BATT_EMPTY_V         3.2f
#define TX_BATT_FULL_V          4.2f

// Target service and characteristic UUIDs (16-bit short form)
#define FPV_SERVICE_UUID16     0xFF00
#define FPV_TX_CHAR_UUID16     0xFF01
#define FPV_RX_CHAR_UUID16     0xFF02  // Telemetry characteristic (notify)
#define FPV_PAIR_CHAR_UUID16   0xFF03  // Pairing key characteristic

// Pairing key size
#define PAIRING_KEY_SIZE 16

// EEPROM addresses
#define EEPROM_SIZE 512
#define BOND_MAGIC_ADDR 99
#define BOND_MAC_ADDR 100
#define BOND_MAGIC_VALUE 0xAA
// Calibration storage
#define CAL_MAGIC_ADDR 150
#define CAL_DATA_ADDR 152
#define CAL_MAGIC_VALUE 0xC5

// Connection states
typedef enum {
    STATE_IDLE,
    STATE_SCANNING,
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_DISCOVERING_SERVICES,
    STATE_DISCOVERING_CHARACTERISTICS,
    STATE_READY,
    STATE_ERROR
} ble_state_t;

// Menu states
typedef enum {
    MENU_NORMAL_OPERATION,
    MENU_MAIN,
    MENU_SCANNING,
    MENU_SELECT_DEVICE,
    MENU_CONNECTING,
    MENU_PAIRING,         // Pairing in progress
    MENU_CALIBRATION      // Stick calibration
} menu_state_t;

// Discovered device info
typedef struct {
    bd_addr_t address;
    bd_addr_type_t address_type;
    int8_t rssi;
    char name[32];
    bool has_fpv_service;
} discovered_device_t;

// Application state
static ble_state_t ble_state = STATE_IDLE;
static menu_state_t menu_state = MENU_NORMAL_OPERATION;

// BTstack objects
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_timer_source_t scan_timer;

// Connection state
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static bool connected = false;

// GATT client state
static gatt_client_service_t fpv_service;
static gatt_client_characteristic_t tx_characteristic;
static gatt_client_characteristic_t rx_characteristic;
static gatt_client_characteristic_t pair_characteristic;
static bool service_found = false;
static bool characteristic_found = false;
static bool rx_characteristic_found = false;
static bool pair_characteristic_found = false;
static uint16_t tx_char_value_handle = 0;
static uint16_t rx_char_value_handle = 0;
static uint16_t pair_char_value_handle = 0;
static bool notifications_enabled = false;

// Battery telemetry received from RX
static float battery_voltage = 0.0f;
static float battery_current = 0.0f;
static uint8_t battery_remaining = 0;
static bool battery_data_valid = false;
static unsigned long last_battery_update = 0;

// TX local battery (BQ25620)
static float tx_batt_voltage = 0.0f;
static uint8_t tx_batt_percent = 0;
static uint8_t tx_batt_bars = 0;
static bool tx_batt_valid = false;
static bool tx_batt_low = false;
static unsigned long last_tx_batt_read = 0;
static const unsigned long TX_BATT_READ_INTERVAL = 5000; // ms
static uint16_t tx_batt_raw_last = 0;

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
#define CAL_MOVE_MS   4000
#define CAL_CENTER_MS 500
#define CAL_SPAN_THRESHOLD 500  // minimum span required to accept an axis
static const char* AXIS_LABELS[4] = {"Ail", "Ele", "Rud", "Thr"};
static int current_cal_axis = 0;
static bool cal_enter_event = false;

typedef enum {
    CAL_PHASE_MOVE,
    CAL_PHASE_CENTER,
    CAL_PHASE_DONE
} cal_phase_t;

static cal_phase_t cal_phase = CAL_PHASE_MOVE;
static unsigned long cal_phase_start = 0;
static uint32_t cal_center_accum[4] = {0,0,0,0};
static uint16_t cal_center_samples = 0;

// Pairing state
static bool pairing_in_progress = false;
static bool pairing_scan_mode = false;  // When true, show ALL devices (not just FPV service)
static uint8_t new_pairing_key[PAIRING_KEY_SIZE];

// Discovered devices
#define MAX_DEVICES 10
static discovered_device_t discovered_devices[MAX_DEVICES];
static int discovered_count = 0;
static int selected_device = 0;
static int main_menu_index = 0; // 0=Scan, 1=Exit

// Bonded device
static bd_addr_t bonded_address;
static bd_addr_type_t bonded_address_type = BD_ADDR_TYPE_LE_PUBLIC;
static bool has_bonded_device = false;

// Display
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;  // ADS1115 ADC on I2C bus

// ADS1115 status flag
bool ads_ok = false;

// Security
Security security;
uint16_t sequenceNumber = 0;

// Channel values (mapped to CRSF range 1000-2000)
uint16_t channels[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Raw input values for logging
int16_t raw_adc[4] = {16384, 16384, 16384, 16384}; // Raw ADC values for stick1_x, stick1_y, stick2_x, stick2_y
bool raw_toggles[4] = {false, false, false, false}; // Raw toggle switch states (SW1-SW4)
bool raw_buttons[2] = {false, false}; // Raw button states (ENTER, BACK)

// Timing
unsigned long lastDataSent = 0;
const unsigned long DATA_INTERVAL = 100;  // 20Hz target for lower latency
const uint16_t DESIRED_ATT_MTU = 64;     // Enough for 22B frame + headers
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 100;

// Status string
char status_str[32] = "Idle";

// Forward declarations
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void gatt_client_callback(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void start_scan(void);
static void start_pairing_scan(void);
static void start_scan_internal(bool pairing_mode);
static void stop_scan(void);
static void connect_to_device(int index);
static void connect_to_bonded_device(void);
static void discover_services(void);
static void discover_characteristics(void);
static void send_channel_data(void);
static void load_bonded_device(void);
static void save_bonded_device(void);
static void start_pairing(void);
static void send_pairing_key(void);
static void generate_pairing_key(void);
static void enable_notifications(void);
static bool bq25620_begin(void);
static bool bq25620_read_voltage(float &voltage_out);
static void update_tx_battery(void);
static uint8_t voltage_to_percent(float v);
static uint8_t percent_to_bars(uint8_t pct);
static void load_calibration(void);
static void save_calibration(void);
static void start_calibration(void);
static void process_calibration(void);

// EEPROM functions
static void load_bonded_device(void) {
    EEPROM.begin(EEPROM_SIZE);
    
    if (EEPROM.read(BOND_MAGIC_ADDR) != BOND_MAGIC_VALUE) {
        has_bonded_device = false;
        Serial.println("No bonded device found");
        return;
    }
    
    for (int i = 0; i < 6; i++) {
        bonded_address[i] = EEPROM.read(BOND_MAC_ADDR + i);
    }
    
    has_bonded_device = true;
    Serial.print("Loaded bonded device: ");
    Serial.println(bd_addr_to_str(bonded_address));
}

static void save_bonded_device(void) {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.write(BOND_MAGIC_ADDR, BOND_MAGIC_VALUE);
    
    for (int i = 0; i < 6; i++) {
        EEPROM.write(BOND_MAC_ADDR + i, bonded_address[i]);
    }
    
    EEPROM.commit();
    has_bonded_device = true;
    Serial.println("Bonded device saved");
}

// ============================================================
// Pairing Functions
// ============================================================
static void generate_pairing_key(void) {
    // Generate cryptographically random key using analog noise + multiple entropy sources
    randomSeed(analogRead(A0) ^ micros() ^ (millis() * 1103515245));
    
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        // Mix multiple entropy sources
        new_pairing_key[i] = random(256) ^ (micros() & 0xFF) ^ (analogRead(A0) & 0xFF);
    }
    
    Serial.println("Generated new pairing key:");
    Serial.print("  Key: ");
    for (int i = 0; i < PAIRING_KEY_SIZE; i++) {
        if (new_pairing_key[i] < 0x10) Serial.print("0");
        Serial.print(new_pairing_key[i], HEX);
    }
    Serial.println();
}

static void start_pairing(void) {
    if (!pair_characteristic_found || pair_char_value_handle == 0) {
        Serial.println("ERROR: Pairing characteristic not found on RX!");
        Serial.println("Make sure RX is in pairing mode (hold BOOTSEL 5 sec)");
        strcpy(status_str, "No pair char");
        return;
    }
    
    pairing_in_progress = true;
    menu_state = MENU_PAIRING;
    strcpy(status_str, "Pairing...");
    
    // Generate new pairing key
    generate_pairing_key();
    
    // Send it to RX
    send_pairing_key();
}

static void send_pairing_key(void) {
    if (!connected || !pair_characteristic_found || pair_char_value_handle == 0) {
        Serial.println("Cannot send pairing key - not connected or characteristic not found");
        pairing_in_progress = false;
        return;
    }
    
    Serial.print("Sending pairing key to RX (handle 0x");
    Serial.print(pair_char_value_handle, HEX);
    Serial.println(")...");
    
    // Send pairing key via GATT write (with response for reliability)
    uint8_t status = gatt_client_write_value_of_characteristic(
        gatt_client_callback,
        connection_handle,
        pair_char_value_handle,
        PAIRING_KEY_SIZE,
        new_pairing_key
    );
    
    if (status != ERROR_CODE_SUCCESS) {
        Serial.print("ERROR: Failed to send pairing key, status: ");
        Serial.println(status);
        strcpy(status_str, "Pair failed");
        pairing_in_progress = false;
    }
}

// Scan timeout callback
static void scan_timeout_handler(btstack_timer_source_t *ts) {
    (void)ts;
    Serial.println("Scan timeout");
    stop_scan();
    
    if (discovered_count > 0) {
        menu_state = MENU_SELECT_DEVICE;
        strcpy(status_str, "Select device");
    } else {
        menu_state = MENU_MAIN;
        strcpy(status_str, "No devices");
    }
}

// Start BLE scanning
static void start_scan(void) {
    start_scan_internal(false);
}

// Start BLE scanning for pairing (shows all devices)
static void start_pairing_scan(void) {
    start_scan_internal(true);
}

static void start_scan_internal(bool pairing_mode) {
    pairing_scan_mode = pairing_mode;
    
    if (pairing_mode) {
        Serial.println("Starting PAIRING scan (all devices)...");
        strcpy(status_str, "Pair scan...");
    } else {
        Serial.println("Starting BLE scan...");
        strcpy(status_str, "Scanning...");
    }
    
    ble_state = STATE_SCANNING;
    
    // Reset discovered devices
    discovered_count = 0;
    selected_device = 0;
    
    // Set scan parameters
    gap_set_scan_parameters(1, 0x0030, 0x0030);  // Active scan, 30ms interval/window
    gap_start_scan();
    
    // Set scan timeout (10 seconds)
    btstack_run_loop_set_timer(&scan_timer, 10000);
    btstack_run_loop_set_timer_handler(&scan_timer, scan_timeout_handler);
    btstack_run_loop_add_timer(&scan_timer);
}

// Stop BLE scanning
static void stop_scan(void) {
    Serial.println("Stopping scan");
    gap_stop_scan();
    btstack_run_loop_remove_timer(&scan_timer);
    ble_state = STATE_IDLE;
    pairing_scan_mode = false;
}

// Connect to discovered device
static void connect_to_device(int index) {
    if (index >= discovered_count) return;
    
    Serial.print("Connecting to: ");
    Serial.println(bd_addr_to_str(discovered_devices[index].address));
    
    ble_state = STATE_CONNECTING;
    menu_state = MENU_CONNECTING;
    strcpy(status_str, "Connecting...");
    
    // Copy address for bonding
    memcpy(bonded_address, discovered_devices[index].address, 6);
    bonded_address_type = discovered_devices[index].address_type;
    
    // Connect
    gap_connect(discovered_devices[index].address, discovered_devices[index].address_type);
}

// Connect to bonded device
static void connect_to_bonded_device(void) {
    if (!has_bonded_device) return;
    
    Serial.print("Connecting to bonded device: ");
    Serial.println(bd_addr_to_str(bonded_address));
    
    ble_state = STATE_CONNECTING;
    menu_state = MENU_CONNECTING;
    strcpy(status_str, "Auto-connect...");
    
    gap_connect(bonded_address, bonded_address_type);
}

// Discover GATT services
static void discover_services(void) {
    Serial.println("Discovering services...");
    ble_state = STATE_DISCOVERING_SERVICES;
    strcpy(status_str, "Finding service");
    
    service_found = false;
    
    // Discover all services
    gatt_client_discover_primary_services(gatt_client_callback, connection_handle);
}

// Discover characteristics for FPV service
static void discover_characteristics(void) {
    Serial.println("Discovering characteristics...");
    ble_state = STATE_DISCOVERING_CHARACTERISTICS;
    strcpy(status_str, "Finding chars");
    
    characteristic_found = false;
    rx_characteristic_found = false;
    
    // Discover characteristics for our service
    gatt_client_discover_characteristics_for_service(gatt_client_callback, connection_handle, &fpv_service);
}

// Enable notifications on RX characteristic for battery telemetry
static void enable_notifications(void) {
    if (!rx_characteristic_found || rx_char_value_handle == 0) {
        Serial.println("Cannot enable notifications - RX characteristic not found");
        return;
    }
    
    Serial.println("Enabling notifications on RX characteristic...");
    
    // Write to Client Characteristic Configuration Descriptor (CCCD) to enable notifications
    // CCCD value 0x0001 = enable notifications
    uint8_t status = gatt_client_write_client_characteristic_configuration(
        gatt_client_callback,
        connection_handle,
        &rx_characteristic,
        GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION
    );
    
    if (status == ERROR_CODE_SUCCESS) {
        Serial.println("Notification enable request sent");
        notifications_enabled = true;
    } else {
        Serial.print("Failed to enable notifications, status: ");
        Serial.println(status);
    }
}

// Send channel data
static void send_channel_data(void) {
    if (!connected || !characteristic_found || tx_char_value_handle == 0) {
        return;
    }
    
    if (millis() - lastDataSent < DATA_INTERVAL) {
        return;
    }
    lastDataSent = millis();
    
    // Increment sequence
    sequenceNumber++;
    if (sequenceNumber == 0) sequenceNumber = 1;
    
    // Encode frame
    uint8_t frame[FRAME_SIZE];
    Protocol::encodeFrame(channels, sequenceNumber, &security, frame);
    
    // Send via GATT write without response
    uint8_t status = gatt_client_write_value_of_characteristic_without_response(
        connection_handle,
        tx_char_value_handle,
        FRAME_SIZE,
        frame
    );
    
    // Debug occasionally
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 2000) {
        Serial.print("Sent seq ");
        Serial.print(sequenceNumber);
        Serial.print(" to handle 0x");
        Serial.print(tx_char_value_handle, HEX);
        Serial.print(", status: ");
        Serial.println(status);
        lastDebug = millis();
    }
}

// GATT client callback
static void gatt_client_callback(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    
    if (packet_type != HCI_EVENT_PACKET) return;
    
    uint8_t event_type = hci_event_packet_get_type(packet);
    
    switch (event_type) {
        case GATT_EVENT_SERVICE_QUERY_RESULT: {
            gatt_client_service_t service;
            gatt_event_service_query_result_get_service(packet, &service);
            
            Serial.print("Service found: ");
            Serial.print(service.uuid16, HEX);
            Serial.print(" (handles 0x");
            Serial.print(service.start_group_handle, HEX);
            Serial.print(" - 0x");
            Serial.print(service.end_group_handle, HEX);
            Serial.println(")");
            
            // Check if this is our FPV service
            if (service.uuid16 == FPV_SERVICE_UUID16) {
                Serial.println("*** FPV Service found! ***");
                service_found = true;
                fpv_service = service;
            }
            break;
        }
        
        case GATT_EVENT_QUERY_COMPLETE: {
            uint8_t att_status = gatt_event_query_complete_get_att_status(packet);
            Serial.print("Query complete, status: ");
            Serial.println(att_status);
            
            if (ble_state == STATE_DISCOVERING_SERVICES) {
                if (service_found) {
                    // Service found, now discover characteristics
                    discover_characteristics();
                } else {
                    Serial.println("FPV service not found!");
                    strcpy(status_str, "No FPV service");
                    ble_state = STATE_ERROR;
                }
            } else if (ble_state == STATE_DISCOVERING_CHARACTERISTICS) {
                if (characteristic_found) {
                    Serial.println("*** Ready to send data! ***");
                    Serial.print("TX handle: 0x");
                    Serial.println(tx_char_value_handle, HEX);
                    
                    if (rx_characteristic_found) {
                        Serial.print("RX handle (telemetry): 0x");
                        Serial.println(rx_char_value_handle, HEX);
                        // Enable notifications for battery telemetry
                        enable_notifications();
                    } else {
                        Serial.println("Note: RX telemetry characteristic not found");
                    }
                    
                    if (pair_characteristic_found) {
                        Serial.print("Pair handle: 0x");
                        Serial.println(pair_char_value_handle, HEX);
                        Serial.println("RX supports pairing - can send new pairing key");
                    } else {
                        Serial.println("Note: Pairing characteristic not found (older RX?)");
                    }
                    
                    ble_state = STATE_READY;
                    menu_state = MENU_NORMAL_OPERATION;
                    connected = true;
                    strcpy(status_str, "Ready");
                    
                    // Save bonded device (this may block briefly)
                    Serial.println("Saving bonded device...");
                    save_bonded_device();
                    Serial.println("Bonded device saved, continuing...");
                    
                    // Force display update on next loop iteration
                    lastDisplayUpdate = 0;
                    Serial.println("Ready state set, display will update in loop()");
                } else {
                    Serial.println("TX characteristic not found!");
                    strcpy(status_str, "No TX char");
                    ble_state = STATE_ERROR;
                }
            }
            
            // Handle pairing write completion
            if (pairing_in_progress && att_status == 0) {
                Serial.println("*** PAIRING SUCCESSFUL ***");
                
                // Save the new pairing key locally
                if (security.setPairingKey(new_pairing_key)) {
                    Serial.println("Pairing key saved to TX EEPROM");
                    strcpy(status_str, "Paired!");
                    
                    // Reset sequence number for fresh start
                    sequenceNumber = 0;
                } else {
                    Serial.println("ERROR: Failed to save pairing key locally!");
                    strcpy(status_str, "Save failed");
                }
                
                pairing_in_progress = false;
                menu_state = MENU_NORMAL_OPERATION;
                
                // Force display update
                lastDisplayUpdate = 0;
            } else if (pairing_in_progress && att_status != 0) {
                Serial.print("*** PAIRING FAILED *** ATT error: ");
                Serial.println(att_status);
                strcpy(status_str, "Pair rejected");
                pairing_in_progress = false;
                menu_state = MENU_NORMAL_OPERATION;
            }
            break;
        }
        
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            gatt_client_characteristic_t characteristic;
            gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
            
            Serial.print("Characteristic found: 0x");
            Serial.print(characteristic.uuid16, HEX);
            Serial.print(", value handle: 0x");
            Serial.print(characteristic.value_handle, HEX);
            Serial.print(", properties: 0x");
            Serial.println(characteristic.properties, HEX);
            
            // Check if this is our TX characteristic (channel data)
            if (characteristic.uuid16 == FPV_TX_CHAR_UUID16) {
                Serial.println("*** TX Characteristic found! ***");
                characteristic_found = true;
                tx_characteristic = characteristic;
                tx_char_value_handle = characteristic.value_handle;
            }
            
            // Check if this is the RX characteristic (telemetry notifications)
            if (characteristic.uuid16 == FPV_RX_CHAR_UUID16) {
                Serial.println("*** RX Characteristic found (telemetry)! ***");
                rx_characteristic_found = true;
                rx_characteristic = characteristic;
                rx_char_value_handle = characteristic.value_handle;
            }
            
            // Check if this is the Pairing characteristic
            if (characteristic.uuid16 == FPV_PAIR_CHAR_UUID16) {
                Serial.println("*** Pairing Characteristic found! ***");
                pair_characteristic_found = true;
                pair_characteristic = characteristic;
                pair_char_value_handle = characteristic.value_handle;
            }
            break;
        }
        
        case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT: {
            // Handle write completion for pairing
            if (pairing_in_progress) {
                Serial.println("Pairing key write acknowledged");
            }
            break;
        }
        
        case GATT_EVENT_NOTIFICATION: {
            // Handle incoming notification (battery telemetry from RX)
            uint16_t value_handle = gatt_event_notification_get_value_handle(packet);
            uint16_t value_length = gatt_event_notification_get_value_length(packet);
            const uint8_t *value = gatt_event_notification_get_value(packet);
            
            // Check if this is from our RX characteristic
            if (value_handle == rx_char_value_handle && value_length == 5) {
                // Parse battery telemetry: voltage*10 (16-bit BE), current*10 (16-bit BE), remaining (8-bit)
                uint16_t voltage_x10 = (value[0] << 8) | value[1];
                uint16_t current_x10 = (value[2] << 8) | value[3];
                
                battery_voltage = voltage_x10 / 10.0f;
                battery_current = current_x10 / 10.0f;
                battery_remaining = value[4];
                battery_data_valid = true;
                last_battery_update = millis();
                
                Serial.print("[TELEM] Battery: ");
                Serial.print(battery_voltage, 1);
                Serial.print("V ");
                Serial.print(battery_current, 1);
                Serial.print("A ");
                Serial.print(battery_remaining);
                Serial.println("%");
            }
            break;
        }
        
        default:
            break;
    }
}

// Main HCI packet handler
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    
    if (packet_type != HCI_EVENT_PACKET) return;
    
    uint8_t event_type = hci_event_packet_get_type(packet);
    
    switch (event_type) {
        case BTSTACK_EVENT_STATE:
            // BTstack state changed - just log it
            Serial.print("[BLE] BTstack state: ");
            Serial.println(btstack_event_state_get_state(packet));
            break;
            
        case GAP_EVENT_ADVERTISING_REPORT: {
            bd_addr_t addr;
            gap_event_advertising_report_get_address(packet, addr);
            bd_addr_type_t addr_type = (bd_addr_type_t)gap_event_advertising_report_get_address_type(packet);
            int8_t rssi = gap_event_advertising_report_get_rssi(packet);
            uint8_t length = gap_event_advertising_report_get_data_length(packet);
            const uint8_t *data = gap_event_advertising_report_get_data(packet);
            
            // Check if this device advertises our FPV service
            bool has_fpv_service = false;
            char name[32] = "Unknown";
            
            // Parse advertisement data
            ad_context_t context;
            for (ad_iterator_init(&context, length, data); ad_iterator_has_more(&context); ad_iterator_next(&context)) {
                uint8_t data_type = ad_iterator_get_data_type(&context);
                uint8_t data_len = ad_iterator_get_data_len(&context);
                const uint8_t *ad_data = ad_iterator_get_data(&context);
                
                switch (data_type) {
                    case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
                    case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
                        for (int i = 0; i < data_len; i += 2) {
                            uint16_t uuid16 = little_endian_read_16(ad_data, i);
                            if (uuid16 == FPV_SERVICE_UUID16) {
                                has_fpv_service = true;
                            }
                        }
                        break;
                        
                    case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
                    case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
                        memcpy(name, ad_data, data_len < 31 ? data_len : 31);
                        name[data_len < 31 ? data_len : 31] = '\0';
                        break;
                }
            }
            
            // Debug: log all devices seen (first time only)
            static uint8_t seen_devices[20][6];
            static int seen_count = 0;
            bool first_time = true;
            for (int i = 0; i < seen_count && i < 20; i++) {
                if (memcmp(seen_devices[i], addr, 6) == 0) {
                    first_time = false;
                    break;
                }
            }
            if (first_time && seen_count < 20) {
                memcpy(seen_devices[seen_count], addr, 6);
                seen_count++;
                Serial.print("[SCAN] Device: ");
                Serial.print(name);
                Serial.print(" (");
                Serial.print(bd_addr_to_str(addr));
                Serial.print(") RSSI:");
                Serial.print(rssi);
                Serial.print(" FPV:");
                Serial.println(has_fpv_service ? "YES" : "no");
            }
            
            // Auto-connect if this is our bonded device (only in normal scan mode)
            if (has_bonded_device && ble_state == STATE_SCANNING && !pairing_scan_mode) {
                if (memcmp(addr, bonded_address, 6) == 0) {
                    Serial.println("Found bonded device, connecting...");
                    stop_scan();
                    gap_connect(addr, addr_type);
                    ble_state = STATE_CONNECTING;
                    strcpy(status_str, "Found bond");
                    return;
                }
            }
            
            // In pairing scan mode, show ALL devices with names
            // In normal scan mode, only show FPV service devices
            bool should_add = false;
            if (pairing_scan_mode) {
                // In pairing mode, add any device with a name (not "Unknown")
                // Prioritize FPV service devices
                should_add = (strcmp(name, "Unknown") != 0) || has_fpv_service;
            } else {
                // Normal mode - only FPV service devices
                should_add = has_fpv_service;
            }
            
            if (should_add && discovered_count < MAX_DEVICES) {
                // Check if already discovered
                bool already_found = false;
                for (int i = 0; i < discovered_count; i++) {
                    if (memcmp(discovered_devices[i].address, addr, 6) == 0) {
                        discovered_devices[i].rssi = rssi;
                        already_found = true;
                        break;
                    }
                }
                
                if (!already_found) {
                    Serial.print(has_fpv_service ? "FPV Device: " : "BLE Device: ");
                    Serial.print(name);
                    Serial.print(" (");
                    Serial.print(bd_addr_to_str(addr));
                    Serial.print(") RSSI: ");
                    Serial.println(rssi);
                    
                    memcpy(discovered_devices[discovered_count].address, addr, 6);
                    discovered_devices[discovered_count].address_type = addr_type;
                    discovered_devices[discovered_count].rssi = rssi;
                    strncpy(discovered_devices[discovered_count].name, name, 31);
                    discovered_devices[discovered_count].has_fpv_service = has_fpv_service;
                    discovered_count++;
                }
            }
            break;
        }
        
        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE: {
                    uint8_t status = hci_subevent_le_connection_complete_get_status(packet);
                    if (status == 0) {
                        connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                        connected = true;
                        
                        Serial.print("[BLE] Connected, handle: 0x");
                        Serial.println(connection_handle, HEX);
                        strcpy(status_str, "Connected");
                        
            // Request MTU negotiation to fit frames efficiently
            gatt_client_send_mtu_negotiation(gatt_client_callback, connection_handle);
                        // Start service discovery
                        discover_services();
                    } else {
                        Serial.print("[BLE] Connection failed: ");
                        Serial.println(status);
                        ble_state = STATE_ERROR;
                        strcpy(status_str, "Conn failed");
                    }
                    break;
                }
            }
            break;
            
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            Serial.println("[BLE] Disconnected");
            connection_handle = HCI_CON_HANDLE_INVALID;
            connected = false;
            characteristic_found = false;
            rx_characteristic_found = false;
            pair_characteristic_found = false;
            service_found = false;
            tx_char_value_handle = 0;
            rx_char_value_handle = 0;
            pair_char_value_handle = 0;
            pairing_in_progress = false;
            notifications_enabled = false;
            battery_data_valid = false;
            ble_state = STATE_IDLE;
            strcpy(status_str, "Disconnected");
            
            // Try to reconnect if we have a bonded device
            if (has_bonded_device) {
                delay(1000);
                connect_to_bonded_device();
            }
            break;
            
        default:
            break;
    }
}

// Read input devices
void readInputs() {
    // Read analog sticks from ADS1115 (16-bit ADC: 0-32767)
    // Use default center values if ADS1115 isn't available
    int16_t stick1_x = 16384;  // Center value (32767/2)
    int16_t stick1_y = 16384;
    int16_t stick2_x = 16384;
    int16_t stick2_y = 16384;
    
    // Always read from ADS1115 if initialized (for logging purposes)
    // Only use values for channel mapping when in normal operation
    if (ads_ok) {
        // Read from ADS1115 - Adafruit library handles I2C communication
        // These reads are fast (~1ms each) and shouldn't block
        // If ADS1115 isn't responding, reads will return last value or 0
        // Channel mapping: A,E,R,T from ADS1115 channels 0,1,2,3
        stick1_x = ads.readADC_SingleEnded(AILERON_CHANNEL);   // Aileron
        stick1_y = ads.readADC_SingleEnded(ELEVATOR_CHANNEL);  // Elevator
        stick2_x = ads.readADC_SingleEnded(RUDDER_CHANNEL);    // Rudder
        stick2_y = ads.readADC_SingleEnded(THROTTLE_CHANNEL);  // Throttle
        
        // Store raw ADC values for logging (always update these)
        raw_adc[0] = stick1_x;
        raw_adc[1] = stick1_y;
        raw_adc[2] = stick2_x;
        raw_adc[3] = stick2_y;
    } else {
        // Store default center values when ADS1115 not available
        raw_adc[0] = stick1_x;
        raw_adc[1] = stick1_y;
        raw_adc[2] = stick2_x;
        raw_adc[3] = stick2_y;
    }
    
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
    // Calibrated based on actual ADC readings:
    // ADC range appears to be ~2917 (min) to ~23420 (max), center ~13199
    // Map using calibration (even during calibration for live view)
    auto mapCal = [](int16_t val, uint16_t vmin, uint16_t vmax) -> uint16_t {
        if (vmax <= vmin + 10) return 1500; // avoid div0
        return (uint16_t)constrain(map(val, vmin, vmax, 1000, 2000), 1000, 2000);
    };
    auto mapCalInverted = [](int16_t val, uint16_t vmin, uint16_t vmax) -> uint16_t {
        if (vmax <= vmin + 10) return 1500; // avoid div0
        // Invert mapping so ADC min -> CRSF max for this channel
        return (uint16_t)constrain(map(val, vmin, vmax, 2000, 1000), 1000, 2000);
    };
    channels[0] = mapCalInverted(stick1_x, calib.min[0], calib.max[0]);  // Aileron (invert)
    channels[1] = mapCal(stick1_y, calib.min[1], calib.max[1]);  // Elevator
    channels[2] = mapCal(stick2_x, calib.min[2], calib.max[2]);  // Rudder
    channels[3] = mapCal(stick2_y, calib.min[3], calib.max[3]);  // Throttle

    // Apply EdgeTX-style dual-rate/expo (Radiomaster Pocket defaults)
    auto applyCurve = [](uint16_t us, float rate, float expo) -> uint16_t {
        float x = (int(us) - 1500) / 500.0f;                  // -1..1
        float x_expo = (1.0f - expo) * x + expo * x * x * x;  // soften center
        float scaled = x_expo * rate;                         // cap max throw
        int out = int(1500 + scaled * 500);
        return (uint16_t)constrain(out, 1000, 2000);
    };
    // Typical EdgeTX wizard defaults: ~70% rate, 30% expo on sticks
    // (throttle stays linear)
    const float RATE = 0.7f;
    const float EXPO = 0.3f;
    channels[0] = applyCurve(channels[0], RATE, EXPO);
    channels[1] = applyCurve(channels[1], RATE, EXPO);
    channels[3] = applyCurve(channels[3], RATE, EXPO);
    // Limit CH0/CH1 travel to 1300-1700 with center 1500
    channels[0] = (uint16_t)constrain(channels[0], 1300, 1700);
    channels[1] = (uint16_t)constrain(channels[1], 1300, 1700);
    
    if (menu_state == MENU_NORMAL_OPERATION) {
        
        // Read toggle switches (active low - LOW = switch engaged)
        bool sw1 = digitalRead(TOGGLE_SWITCH1_PIN) == LOW;
        bool sw2 = digitalRead(TOGGLE_SWITCH2_PIN) == LOW;
        bool sw3 = digitalRead(TOGGLE_SWITCH3_PIN) == LOW;
        bool sw4 = digitalRead(TOGGLE_SWITCH4_PIN) == LOW;
        
        // Store raw toggle states for logging
        raw_toggles[0] = sw1;
        raw_toggles[1] = sw2;
        raw_toggles[2] = sw3;
        raw_toggles[3] = sw4;
        
        channels[4] = sw1 ? 2000 : 1000;
        channels[5] = sw2 ? 2000 : 1000;
        channels[6] = sw3 ? 2000 : 1000;
        channels[7] = sw4 ? 2000 : 1000;
        
        // Log toggle switch states when they change
        static bool lastSw1 = false, lastSw2 = false, lastSw3 = false, lastSw4 = false;
        static unsigned long lastSwitchLog = 0;
        
        if (sw1 != lastSw1 || sw2 != lastSw2 || sw3 != lastSw3 || sw4 != lastSw4) {
            Serial.print("[SWITCHES] ");
            Serial.print("SW1(GP" + String(TOGGLE_SWITCH1_PIN) + "):" + String(sw1 ? "ON" : "OFF") + " ");
            Serial.print("SW2(GP" + String(TOGGLE_SWITCH2_PIN) + "):" + String(sw2 ? "ON" : "OFF") + " ");
            Serial.print("SW3(GP" + String(TOGGLE_SWITCH3_PIN) + "):" + String(sw3 ? "ON" : "OFF") + " ");
            Serial.print("SW4(GP" + String(TOGGLE_SWITCH4_PIN) + "):" + String(sw4 ? "ON" : "OFF"));
            Serial.print(" -> Ch4:" + String(channels[4]) + " Ch5:" + String(channels[5]) + " Ch6:" + String(channels[6]) + " Ch7:" + String(channels[7]));
            Serial.println();
            lastSw1 = sw1;
            lastSw2 = sw2;
            lastSw3 = sw3;
            lastSw4 = sw4;
            lastSwitchLog = millis();
        } else if (millis() - lastSwitchLog > 2000) {
            // Log current state every 2 seconds even if no change
            Serial.print("[SWITCHES] ");
            Serial.print("SW1(GP" + String(TOGGLE_SWITCH1_PIN) + "):" + String(sw1 ? "ON" : "OFF") + " ");
            Serial.print("SW2(GP" + String(TOGGLE_SWITCH2_PIN) + "):" + String(sw2 ? "ON" : "OFF") + " ");
            Serial.print("SW3(GP" + String(TOGGLE_SWITCH3_PIN) + "):" + String(sw3 ? "ON" : "OFF") + " ");
            Serial.print("SW4(GP" + String(TOGGLE_SWITCH4_PIN) + "):" + String(sw4 ? "ON" : "OFF"));
            Serial.println();
            lastSwitchLog = millis();
        }
    }
}

// Update display
void updateDisplay() {
    if (millis() - lastDisplayUpdate < DISPLAY_INTERVAL) return;
    lastDisplayUpdate = millis();
    
    // Clear display buffer
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    
    switch (menu_state) {
        case MENU_NORMAL_OPERATION:
            display.print("Status: ");
            display.println(status_str);
            // TX battery (local)
            if (tx_batt_valid) {
                display.print("TX: ");
                display.print(tx_batt_voltage, 2);
                display.print("V ");
                display.print(tx_batt_percent);
                display.print("% ");
                // 4-bar indicator, blink last bar if low
                bool blink = tx_batt_low && ((millis() / 500) % 2 == 0);
                for (uint8_t i = 0; i < 4; i++) {
                    bool filled = (i < tx_batt_bars);
                    if (tx_batt_low && i == tx_batt_bars - 1) {
                        filled = !blink;  // blink the last bar when low
                    }
                    display.print(filled ? "|" : ".");
                }
                display.println();
            } else {
                display.println("TX: --");
            }
            // RX battery (from telemetry) if available
            if (battery_data_valid) {
                display.print("RX: ");
                display.print(battery_voltage, 1);
                display.print("V ");
                display.print(battery_remaining);
                display.println("%");
            }
            display.print("Ch: ");
            display.print(channels[0]);
            display.print(" ");
            display.print(channels[1]);
            display.print(" ");
            display.print(channels[2]);
            display.print(" ");
            display.println(channels[3]);
            // Connection indicator
            display.fillRect(120, 0, 8, 8, connected ? SH110X_WHITE : SH110X_BLACK);
            display.drawRect(120, 0, 8, 8, SH110X_WHITE);
            break;
            
        case MENU_MAIN:
            display.println("=== MENU ===");
            display.print(main_menu_index == 0 ? ">" : " ");
            display.println("Scan Devices");
            display.print(main_menu_index == 1 ? ">" : " ");
            display.println("Exit");
            display.println("");
            display.println("GP18: Enter");
            display.println("GP13: Back/3s:Pair");
            break;

        case MENU_CALIBRATION:
            display.println("=== CALIB ===");
            display.print(AXIS_LABELS[current_cal_axis]);
            display.print(" Min:");
            display.println(calib.min[current_cal_axis]);
            display.print("Max:");
            display.println(calib.max[current_cal_axis]);
            display.print("Span:");
            display.println((calib.max[current_cal_axis] > calib.min[current_cal_axis]) ? (calib.max[current_cal_axis] - calib.min[current_cal_axis]) : 0);
            display.print("Now:");
            display.println(raw_adc[current_cal_axis]);
            if (cal_phase == CAL_PHASE_MOVE) {
                display.println("Move axis, Enter->next");
            } else if (cal_phase == CAL_PHASE_CENTER) {
                display.println("Centering...");
            } else {
                display.println("Saving...");
            }
            break;
            
        case MENU_SCANNING:
            display.println("Scanning...");
            display.print("Found: ");
            display.println(discovered_count);
            break;
            
        case MENU_SELECT_DEVICE:
            display.println("Select Device:");
            for (int i = 0; i < min(discovered_count, 4); i++) {
                display.print(i == selected_device ? ">" : " ");
                // Show * for FPV service devices
                display.print(discovered_devices[i].has_fpv_service ? "*" : " ");
                display.println(discovered_devices[i].name);
            }
            if (pairing_scan_mode) {
                display.println("(*=FPV service)");
            }
            break;
            
        case MENU_CONNECTING:
            display.println("Connecting...");
            display.println(status_str);
            break;
            
        case MENU_PAIRING:
            display.println("=== PAIRING ===");
            display.println(status_str);
            display.println("");
            display.println("Sending key to RX");
            display.println("Please wait...");
            break;
    }
    
    // Update physical display
    display.display();
    
    // Verify display was updated (check if display buffer was written)
    // Note: This is a best-effort check - display.display() doesn't return error
}

// Handle buttons and joystick navigation
void handleButtons() {
    // Read menu navigation buttons (active high)
    bool enter_btn = digitalRead(ENTER_BUTTON_PIN) == HIGH;
    bool back_btn = digitalRead(BACK_BUTTON_PIN) == HIGH;

    // Detect long hold of both buttons to enter calibration (5s).
    // Run this BEFORE any menu handling and suppress normal button actions
    // while both are held.
    static unsigned long both_hold_start = 0;
    static bool in_both_hold = false;
    if (enter_btn && back_btn) {
        if (!in_both_hold) {
            in_both_hold = true;
            both_hold_start = millis();
        } else if (millis() - both_hold_start >= 5000) {
            Serial.println("Both buttons held 5s - entering calibration");
            start_calibration();
            // prevent retrigger until released
            in_both_hold = false;
            both_hold_start = 0;
        }
        // When both are held, do not process individual button/menu actions
        return;
    } else {
        in_both_hold = false;
        both_hold_start = 0;
    }

    // If in calibration, only use Enter to advance; ignore other menu handling.
    if (menu_state == MENU_CALIBRATION) {
        static bool enter_pressed_cal = false;
        if (enter_btn && !enter_pressed_cal) {
            enter_pressed_cal = true;
        } else if (!enter_btn && enter_pressed_cal) {
            enter_pressed_cal = false;
            cal_enter_event = true; // signal to calibration state machine
        }
        // Optional: allow Back to abort calibration (not required now)
        return;
    }
    
    // Store raw button states for logging
    raw_buttons[0] = enter_btn;
    raw_buttons[1] = back_btn;
    
    // When connected and transmitting, require both buttons to enter menu
    static unsigned long both_buttons_press_start = 0;
    static bool both_buttons_pressed = false;
    
    if (connected && menu_state == MENU_NORMAL_OPERATION) {
        bool both_pressed = enter_btn && back_btn;
        if (both_pressed && !both_buttons_pressed) {
            both_buttons_pressed = true;
            both_buttons_press_start = millis();
        } else if (!both_pressed && both_buttons_pressed) {
            both_buttons_pressed = false;
            unsigned long duration = millis() - both_buttons_press_start;
            
            // Require both buttons held for at least 500ms to prevent accidental activation
            if (duration >= 500) {
                Serial.println("Both buttons pressed - entering menu");
                menu_state = MENU_MAIN;
                main_menu_index = 0;
            }
        }
        // Don't process individual buttons when connected and transmitting
        return;
    }
    
    // Joystick navigation - delta-based (loose)
    const int16_t NAV_CENTER_DEADZONE = 400;   // must be outside this deadzone from center
    const int16_t NAV_DELTA_THRESHOLD = 600;   // delta needed to trigger another move
    const unsigned long NAV_DEBOUNCE = 120;    // ms between navigation events
    
    static int16_t last_nav_value = 0;  // Track last position that triggered navigation
    static unsigned long last_nav = 0;
    
    // Get joystick values for navigation (use stick2_x for up/down, fallback to stick1_y)
    // Use already-read raw_adc values instead of reading ADC again
    int16_t nav_y = 0;
    if (ads_ok) {
        // Convert to centered values (-16384 to +16384, center at 0)
        // raw_adc[1] = stick1_y, raw_adc[2] = stick2_x
        int16_t stick1_y_centered = raw_adc[1] - 16384;
        int16_t stick2_x_centered = raw_adc[2] - 16384;
        
        // Prefer stick2_x (v2_x) for navigation, but use stick1_y if stick2_x is not deflected enough
        if (abs(stick2_x_centered) > NAV_CENTER_DEADZONE) {
            nav_y = stick2_x_centered;
        } else if (abs(stick1_y_centered) > NAV_CENTER_DEADZONE) {
            nav_y = stick1_y_centered;
        }
    }
    
    // Reset tracking when stick returns to center deadzone
    if (abs(nav_y) <= NAV_CENTER_DEADZONE) {
        last_nav_value = 0;
    }
    
    // Handle menu navigation with joystick - delta-based
    // Only navigate if position changed enough from last trigger point
    int16_t delta = nav_y - last_nav_value;
    
    if (abs(nav_y) > NAV_CENTER_DEADZONE && 
        abs(delta) > NAV_DELTA_THRESHOLD && 
        (millis() - last_nav > NAV_DEBOUNCE)) {
        
        if (menu_state == MENU_SELECT_DEVICE && discovered_count > 0) {
            if (delta > 0) {
                // Moving positive direction (down/right)
                selected_device = (selected_device + 1) % discovered_count;
            } else {
                // Moving negative direction (up/left)
                selected_device = (selected_device - 1 + discovered_count) % discovered_count;
            }
            last_nav = millis();
            last_nav_value = nav_y;  // Remember this position as the new reference
        } else if (menu_state == MENU_MAIN) {
            // Two items: 0=Scan, 1=Exit
            if (delta > 0) {
                main_menu_index = min(main_menu_index + 1, 1);
            } else {
                main_menu_index = max(main_menu_index - 1, 0);
            }
            last_nav = millis();
            last_nav_value = nav_y;
        }
    }
    
    // Enter button (GP18, active high)
    static unsigned long last_enter = 0;
    static bool enter_pressed = false;
    
    if (enter_btn && !enter_pressed) {
        enter_pressed = true;
    } else if (!enter_btn && enter_pressed) {
        enter_pressed = false;
        if (millis() - last_enter > 300) {  // Debounce
            last_enter = millis();
            
            if (menu_state == MENU_SELECT_DEVICE && discovered_count > 0) {
                connect_to_device(selected_device);
            } else if (menu_state == MENU_MAIN) {
                if (main_menu_index == 0) {
                    menu_state = MENU_SCANNING;
                    start_scan();
                } else {
                    menu_state = MENU_NORMAL_OPERATION;
                }
            }
        }
    }
    
    // Back button (GP13, active high)
    static unsigned long back_press_start = 0;
    static bool back_pressed = false;
    static bool long_press_triggered = false;
    
    if (back_btn && !back_pressed) {
        back_pressed = true;
        back_press_start = millis();
        long_press_triggered = false;
    } else if (back_btn && back_pressed && !long_press_triggered) {
        // Check for long press WHILE button is held (no need to release)
        unsigned long duration = millis() - back_press_start;
        
        if (duration >= 3000) {
            long_press_triggered = true;
            
            // Long press - initiate pairing scan
            if (connected && pair_characteristic_found) {
                Serial.println("Long press back - initiating pairing");
                start_pairing();
            } else {
                Serial.println("Long press back - entering pairing scan mode");
                if (connected) {
                    gap_disconnect(connection_handle);
                }
                menu_state = MENU_SCANNING;
                start_pairing_scan();  // Use pairing scan (shows all devices)
            }
        }
    } else if (!back_btn && back_pressed) {
        back_pressed = false;
        unsigned long duration = millis() - back_press_start;
        
        // Only handle short press if long press wasn't triggered
        if (!long_press_triggered && duration >= 200) {
            // Short press - back
            if (menu_state == MENU_SELECT_DEVICE) {
                menu_state = MENU_MAIN;
            } else if (menu_state == MENU_MAIN) {
                menu_state = MENU_NORMAL_OPERATION;
            } else if (menu_state == MENU_PAIRING) {
                // Can't back out of pairing - wait for it to complete
                Serial.println("Pairing in progress, please wait...");
            }
        }
        long_press_triggered = false;
    }
}

void setup() {
    Serial.begin(115200);
    
    unsigned long start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
    delay(500);
    
    Serial.println("=== FPV Transmitter (BLE - Low Level API) ===");
    
    // Initialize I2C for OLED and ADS1115
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    
    // Start with standard I2C speed (100kHz) for compatibility
    // We'll increase it after both devices are initialized
    Wire.setClock(100000);  // 100kHz standard mode
    Serial.println("I2C bus initialized at 100kHz");
    
    // Scan I2C bus to see what devices are present
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
    } else {
        Serial.print("Found ");
        Serial.print(nDevices);
        Serial.println(" device(s)");
    }
    
    // Initialize OLED first (at standard speed)
    delay(250); // wait for the OLED to power up
    display.begin(I2C_ADDRESS, false); // Address 0x3C default, false = no splash screen
    Serial.print("OLED initialized at address 0x");
    Serial.println(I2C_ADDRESS, HEX);
    
    // Show kikobot "k." logo
    display.clearDisplay();
    display.setTextSize(3); // Large text for logo
    display.setTextColor(SH110X_WHITE);
    // Center the text horizontally: (128 - (text_width)) / 2
    // Approximate width of "k." at size 3 is ~24 pixels, so center at ~52
    display.setCursor(52, 20); // Centered horizontally, positioned vertically
    display.print("k.");
    display.display();
    delay(2000);
    
    // Show startup message
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("FPV TX Starting...");
    display.display();
    
    // Small delay to ensure I2C bus is ready
    delay(100);
    
    // Initialize ADS1115
    Serial.print("Attempting to initialize ADS1115 at address 0x");
    Serial.println(ADS1115_ADDRESS, HEX);
    
    if (!ads.begin(ADS1115_ADDRESS, &Wire)) {
        Serial.println("Failed to initialize ADS1115");
        Serial.println("Trying alternative addresses...");
        
        // Try alternative addresses (0x49, 0x4A, 0x4B)
        bool found = false;
        uint8_t alt_addresses[] = {0x49, 0x4A, 0x4B};
        for (int i = 0; i < 3; i++) {
            Serial.print("Trying address 0x");
            Serial.println(alt_addresses[i], HEX);
            if (ads.begin(alt_addresses[i], &Wire)) {
                Serial.print("ADS1115 found at address 0x");
                Serial.println(alt_addresses[i], HEX);
                found = true;
                break;
            }
        }
        
        if (!found) {
            Serial.println("ADS1115 not found at any address");
            Serial.println("Check wiring: SDA=GP4, SCL=GP5, VCC=3.3V, GND=GND");
            Serial.println("ADDR pin: GND=0x48, VDD=0x49, SDA=0x4A, SCL=0x4B");
            Serial.println("Will use default center values for sticks");
            ads_ok = false;
        } else {
            ads_ok = true;
        }
    } else {
        Serial.print("ADS1115 initialized successfully at address 0x");
        Serial.println(ADS1115_ADDRESS, HEX);
        ads_ok = true;
    }
    
    if (ads_ok) {
        // Set gain to 1 (default, ±4.096V range)
        // This gives us full 16-bit resolution (0-32767)
        ads.setGain(GAIN_ONE);
        
        // Set data rate to 250 SPS (Samples Per Second)
        // We need to read 4 channels at 50Hz (20ms intervals)
        // Minimum required: 4 channels * 50Hz = 200 SPS
        // Using 250 SPS provides headroom and ensures fast reads
        // Each channel read takes ~4ms at 250 SPS, all 4 channels ~16ms (well within 20ms budget)
        ads.setDataRate(RATE_ADS1115_250SPS);
        Serial.println("ADS1115 configured: Gain=1, DataRate=250 SPS");
        
        // Test read to verify ADC is working
        Serial.println("Testing ADC reads...");
        int16_t test_val = ads.readADC_SingleEnded(0);
        Serial.print("Test read channel 0: ");
        Serial.println(test_val);
        float voltage = (test_val * 4.096) / 32767.0;
        Serial.print("Voltage: ");
        Serial.print(voltage, 3);
        Serial.println("V");
        
        // Now increase I2C speed to 400kHz for faster reads
        // Both devices should be able to handle this after initialization
        Wire.setClock(400000);  // 400kHz fast mode
        Serial.println("I2C bus speed increased to 400kHz");
    }

    // Initialize BQ25620 ADC for TX battery monitoring
    if (bq25620_begin()) {
        Serial.println("BQ25620 ADC enabled");
    } else {
        Serial.println("BQ25620 init failed");
    }
    
    // Initialize menu navigation buttons (active high - use INPUT_PULLDOWN)
    pinMode(ENTER_BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(BACK_BUTTON_PIN, INPUT_PULLDOWN);
    
    // Initialize toggle switches (active low with internal pullup)
    pinMode(TOGGLE_SWITCH1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH2_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH3_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SWITCH4_PIN, INPUT_PULLUP);
    
    // Load bonded device
    load_bonded_device();

    // Load stick calibration (if present)
    load_calibration();
    
    // Initialize security
    if (security.begin()) {
        Serial.println("Security initialized");
    }
    
    // Register HCI event handler BEFORE BTstack.setup()
    Serial.println("Registering event handlers...");
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    
    // Initialize GATT client
    gatt_client_init();
    
    // Use BTstackLib to initialize the Bluetooth stack
    Serial.println("Calling BTstack.setup()...");
    BTstack.setup();
    
    Serial.println("BTstack initialized");
    
    Serial.println("=== TX Ready ===");
    
    // If we have a bonded device, start scanning for it
    if (has_bonded_device) {
        Serial.println("Starting scan for bonded device...");
        strcpy(status_str, "Scanning...");
        start_scan();
    } else {
        strcpy(status_str, "No bond");
        menu_state = MENU_MAIN;
    }
}

void loop() {
    // Process BTstack events using BTstackLib
    BTstack.loop();
    
    // Read inputs
    readInputs();
    
    // Handle buttons
    handleButtons();
    
    // Calibration processing
    if (menu_state == MENU_CALIBRATION) {
        process_calibration();
    }
    
    // Send data if ready
    if (ble_state == STATE_READY && menu_state == MENU_NORMAL_OPERATION) {
        send_channel_data();
    }
    
    // Update display (rate-limited internally)
    updateDisplay();
    
    // Periodically read TX battery (BQ25620)
    update_tx_battery();
    
    // Heartbeat and input logging
    static unsigned long lastHeart = 0;
    if (millis() - lastHeart > 5000) {
        lastHeart = millis();
        Serial.print("TX alive, state: ");
        Serial.print(ble_state);
        Serial.print(", connected: ");
        Serial.print(connected);
        Serial.print(", char_found: ");
        Serial.print(characteristic_found);
        Serial.print(", handle: 0x");
        Serial.println(tx_char_value_handle, HEX);
        
        // Log raw input values (not CRSF mapped)
        Serial.print("[RAW INPUTS] ads_ok="); Serial.print(ads_ok ? "YES" : "NO");
        Serial.print(" menu_state="); Serial.print(menu_state);
        // Analog channels - raw ADC values (0-32767, center ~16384)
        Serial.print(" | Stick1_X:"); Serial.print(raw_adc[0]);
        Serial.print(" Stick1_Y:"); Serial.print(raw_adc[1]);
        Serial.print(" Stick2_X:"); Serial.print(raw_adc[2]);
        Serial.print(" Stick2_Y:"); Serial.print(raw_adc[3]);
        // Also show calculated voltages for debugging
        if (ads_ok) {
            Serial.print(" | V1_X:"); Serial.print((raw_adc[0] * 4.096) / 32767.0, 2);
            Serial.print("V V1_Y:"); Serial.print((raw_adc[1] * 4.096) / 32767.0, 2);
            Serial.print("V V2_X:"); Serial.print((raw_adc[2] * 4.096) / 32767.0, 2);
            Serial.print("V V2_Y:"); Serial.print((raw_adc[3] * 4.096) / 32767.0, 2);
            Serial.print("V");
        }
        // Toggle switches - raw states (GP28, GP21, GP3, GP9)
        Serial.print(" | SW1(GP28):"); Serial.print(raw_toggles[0] ? "ACTIVE" : "INACTIVE");
        Serial.print(" SW2(GP21):"); Serial.print(raw_toggles[1] ? "ACTIVE" : "INACTIVE");
        Serial.print(" SW3(GP3):"); Serial.print(raw_toggles[2] ? "ACTIVE" : "INACTIVE");
        Serial.print(" SW4(GP9):"); Serial.print(raw_toggles[3] ? "ACTIVE" : "INACTIVE");
        // Buttons - raw states (GP18, GP13)
        Serial.print(" | ENTER(GP18):"); Serial.print(raw_buttons[0] ? "ACTIVE" : "INACTIVE");
        Serial.print(" BACK(GP13):"); Serial.print(raw_buttons[1] ? "ACTIVE" : "INACTIVE");
        Serial.println();
    }
    
    delay(1);
}

// ==========================================
// Calibration helpers
// ==========================================

static void load_calibration(void) {
    EEPROM.begin(EEPROM_SIZE);
    if (EEPROM.read(CAL_MAGIC_ADDR) != CAL_MAGIC_VALUE) {
        Serial.println("Calibration not found, using defaults");
        calib_loaded = false;
        return;
    }
    EEPROM.get(CAL_DATA_ADDR, calib);
    // Basic sanity clamp
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

static void save_calibration(void) {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.write(CAL_MAGIC_ADDR, CAL_MAGIC_VALUE);
    EEPROM.put(CAL_DATA_ADDR, calib);
    EEPROM.commit();
    Serial.println("Calibration saved");
}

static void start_calibration(void) {
    menu_state = MENU_CALIBRATION;
    cal_phase = CAL_PHASE_MOVE;
    cal_phase_start = millis();
    current_cal_axis = 0;
    for (int i = 0; i < 4; i++) {
        calib.min[i] = 0xFFFF;
        calib.max[i] = 0;
        calib.center[i] = 0;
        cal_center_accum[i] = 0;
    }
    cal_center_samples = 0;
    strcpy(status_str, "Calibrating");
    lastDisplayUpdate = 0;
}

static void process_calibration(void) {
    unsigned long now = millis();
    int ax = current_cal_axis;

    if (cal_phase == CAL_PHASE_MOVE) {
        // Track min/max for current axis
        uint16_t v = (uint16_t)raw_adc[ax];
        if (v < calib.min[ax]) calib.min[ax] = v;
        if (v > calib.max[ax]) calib.max[ax] = v;

        if (cal_enter_event) {
            cal_enter_event = false;
            uint16_t span = (calib.max[ax] > calib.min[ax]) ? (calib.max[ax] - calib.min[ax]) : 0;
            if (span < CAL_SPAN_THRESHOLD) {
                Serial.println("Cal: span too small, move more");
                strcpy(status_str, "Move more");
                lastDisplayUpdate = 0;
            } else {
                cal_phase = CAL_PHASE_CENTER;
                cal_phase_start = now;
                cal_center_samples = 0;
                cal_center_accum[ax] = 0;
                Serial.println("Cal: center this axis");
                strcpy(status_str, "Centering...");
                lastDisplayUpdate = 0;
            }
        }
    } else if (cal_phase == CAL_PHASE_CENTER) {
        uint16_t v = (uint16_t)raw_adc[ax];
        cal_center_accum[ax] += (uint32_t)v;
        cal_center_samples++;
        if (now - cal_phase_start >= CAL_CENTER_MS) {
            if (cal_center_samples > 0) {
                calib.center[ax] = (uint16_t)(cal_center_accum[ax] / cal_center_samples);
            } else {
                calib.center[ax] = v;
            }
            // Clamp center inside min/max
            if (calib.center[ax] < calib.min[ax]) calib.center[ax] = calib.min[ax];
            if (calib.center[ax] > calib.max[ax]) calib.center[ax] = calib.max[ax];

            // Advance to next axis or finish
            current_cal_axis++;
            if (current_cal_axis >= 4) {
                cal_phase = CAL_PHASE_DONE;
                save_calibration();
                calib_loaded = true;
                menu_state = MENU_NORMAL_OPERATION;
                strcpy(status_str, "Cal done");
                lastDisplayUpdate = 0;
                Serial.println("Cal: completed");
            } else {
                // Reset for next axis
                int na = current_cal_axis;
                calib.min[na] = 0xFFFF;
                calib.max[na] = 0;
                cal_center_accum[na] = 0;
                cal_center_samples = 0;
                cal_phase = CAL_PHASE_MOVE;
                cal_phase_start = now;
                strcpy(status_str, "Next axis");
                lastDisplayUpdate = 0;
                Serial.print("Cal: next axis ");
                Serial.println(AXIS_LABELS[na]);
            }
        }
    }
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
    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    raw &= 0x0FFF;  // VBAT ADC is 12-bit
    // Sanity check: reject impossible values
    if (raw == 0 || raw > 6000) {  // > ~12V is invalid for 1S
        return false;
    }
    raw_out = raw;
    voltage_out = raw * BQ25620_ADC_STEP_V; // volts
    return true;
}

static uint8_t voltage_to_percent(float v) {
    if (v <= TX_BATT_EMPTY_V) return 0;
    if (v >= TX_BATT_FULL_V) return 100;
    float pct = (v - TX_BATT_EMPTY_V) * 100.0f / (TX_BATT_FULL_V - TX_BATT_EMPTY_V);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint8_t)(pct + 0.5f);
}

static uint8_t percent_to_bars(uint8_t pct) {
    if (pct == 0) return 0;
    if (pct <= 25) return 1;
    if (pct <= 50) return 2;
    if (pct <= 75) return 3;
    return 4;
}

static void update_tx_battery(void) {
    // Don't hammer I2C
    if (millis() - last_tx_batt_read < TX_BATT_READ_INTERVAL) return;
    last_tx_batt_read = millis();
    
    float v = 0.0f;
    uint16_t raw = 0;
    if (!bq25620_read_voltage(v, raw)) {
        // Debug once in a while if reads fail
        static unsigned long lastFailLog = 0;
        if (millis() - lastFailLog > 10000) {
            Serial.println("BQ25620 read failed");
            lastFailLog = millis();
        }
        return;
    }
    tx_batt_raw_last = raw;
    Serial.print("[BQ25620] raw:");
    Serial.print(raw);
    Serial.print(" V:");
    Serial.println(v, 3);
    tx_batt_voltage = v;
    tx_batt_percent = voltage_to_percent(v);
    tx_batt_bars = percent_to_bars(tx_batt_percent);
    tx_batt_valid = true;
    tx_batt_low = (v <= TX_BATT_EMPTY_V + 0.05f) || (tx_batt_percent <= 10);
    
    // If critically low at startup, we could halt TX here.
    // Currently we just surface via UI; can be extended to enforce shutdown.
}
