/*
 * BQ2562X Battery Charger Driver for Arduino/RP2040
 * Ported from Zephyr driver
 */

#ifndef BQ2562X_H_
#define BQ2562X_H_

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief BQ2562X I2C address.
 */
#define BQ2562X_I2C_ADDR 0x6A  // Match existing codebase (was 0x6B in original)

/**
 * @name BQ2562X 16-bit Register Addresses
 */
#define BQ2562X_REG_CHARGE_CURRENT_LIMIT       0x02
#define BQ2562X_REG_CHARGE_CURRENT_LIMIT_MASK  0b0000111111000000
#define CHARGE_CURRENT_LIMIT_TO_MA(reg_val)    ((reg_val & BQ2562X_REG_CHARGE_CURRENT_LIMIT_MASK) >> 6)*80
#define MA_TO_CHARGE_CURRENT_LIMIT(ma)         ((ma / 80) << 6) & BQ2562X_REG_CHARGE_CURRENT_LIMIT_MASK
#define MIN_CURRENT_CHARGE_LIMIT               80
#define MAX_CURRENT_CHARGE_LIMIT               3520

#define BQ2562X_REG_CHARGE_VOLTAGE_LIMIT       0x04
#define BQ2562X_REG_CHARGE_VOLTAGE_LIMIT_MASK  0b0000111111111000
#define CHARGE_VOLTAGE_LIMIT_TO_MV(reg_val)    ((reg_val & BQ2562X_REG_CHARGE_VOLTAGE_LIMIT_MASK) >> 3)*10
#define MV_TO_CHARGE_VOLTAGE_LIMIT(mv)         ((mv / 10) << 3) & BQ2562X_REG_CHARGE_VOLTAGE_LIMIT_MASK
#define MIN_CHARGE_VOLTAGE_LIMIT               CHARGE_VOLTAGE_LIMIT_TO_MV(0x015E << 3)
#define MAX_CHARGE_VOLTAGE_LIMIT               CHARGE_VOLTAGE_LIMIT_TO_MV(0x01E0 << 3)

#define BQ2562X_REG_INPUT_CURRENT_LIMIT        0x06
#define BQ2562X_REG_INPUT_CURRENT_LIMIT_MASK   0b0000111111110000
#define INPUT_CURRENT_LIMIT_TO_MA(reg_val)     ((reg_val & BQ2562X_REG_INPUT_CURRENT_LIMIT_MASK) >> 4)*20
#define MA_TO_INPUT_CURRENT_LIMIT(ma)          ((ma / 20) << 4) & BQ2562X_REG_INPUT_CURRENT_LIMIT_MASK
#define MIN_INPUT_CURRENT_LIMIT                INPUT_CURRENT_LIMIT_TO_MA(0x05 << 4)
#define MAX_INPUT_CURRENT_LIMIT                INPUT_CURRENT_LIMIT_TO_MA(0xA0 << 4)

/**
 * @name BQ2562X 8-bit Register Addresses
 */
#define BQ2562X_REG_CHARGE_CONTROL_0         0x14
#define BQ2562X_REG_CHARGE_TIMER_CONTROL     0x15
#define BQ2562X_REG_CHARGER_CONTROL_1        0x16
#define BQ2562X_REG_CHARGER_CONTROL_2        0x17
#define BQ2562X_REG_CHARGER_CONTROL_3        0x18
#define BQ2562X_REG_CHARGER_CONTROL_4        0x19
#define BQ2562X_REG_NTC_CONTROL_0            0x1A
#define BQ2562X_REG_NTC_CONTROL_1            0x1B
#define BQ2562X_REG_NTC_CONTROL_2            0x1C
#define BQ2562X_REG_CHARGER_STATUS_0         0x1D
#define BQ2562X_REG_CHARGER_STATUS_1         0x1E
#define BQ2562X_REG_FAULT_STATUS_0           0x1F
#define BQ2562X_REG_CHARGER_FLAG_0          0x20
#define BQ2562X_REG_PART_INFORMATION         0x38

// ADC Control register definitions
#define BQ2562X_REG_ADC_CONTROL              0x26
#define ADC_CONTROL_EN_POS                   7
#define ADC_CONTROL_EN_MSK                   (1 << ADC_CONTROL_EN_POS)
#define ADC_CONTROL_RATE_POS                 6
#define ADC_CONTROL_RATE_MSK                 (1 << ADC_CONTROL_RATE_POS) // 1=one-shot

// ADC Status register definitions
#define ADC_DONE_STAT_POS                    6
#define ADC_DONE_STAT_MSK                    (1 << ADC_DONE_STAT_POS)

// ADC conversion timeout
#define BQ2562X_ADC_CONVERSION_TIMEOUT_MS 100

// LSB value for VBAT ADC in millivolts
#define BQ2562X_VBAT_ADC_LSB_MV 1.99f

#define BQ2562X_REG_VBAT_ADC                 0x30

// REG0x18 - Charger Control 3 bitfield definitions
#define BATFET_CTRL_POS 0
#define BATFET_CTRL_MSK (0x3 << BATFET_CTRL_POS)

// REG0x14 bitfield definitions
#define CHARGE_CONTROL_0_VRECHG_POS 0
#define CHARGE_CONTROL_0_VRECHG_MSK (1 << CHARGE_CONTROL_0_VRECHG_POS)
#define CHARGE_CONTROL_0_TOPOFF_TMR_POS 3
#define CHARGE_CONTROL_0_TOPOFF_TMR_MSK (0x3 << CHARGE_CONTROL_0_TOPOFF_TMR_POS)

// REG0x16 bitfield definitions
#define CHARGER_CONTROL_1_WATCHDOG_POS 0
#define CHARGER_CONTROL_1_WATCHDOG_MSK (0x3 << CHARGER_CONTROL_1_WATCHDOG_POS)
#define CHARGER_CONTROL_1_WD_RST_POS 2
#define CHARGER_CONTROL_1_WD_RST_MSK (1 << CHARGER_CONTROL_1_WD_RST_POS)
#define CHARGER_CONTROL_1_EN_CHG_POS 5
#define CHARGER_CONTROL_1_EN_CHG_MSK (1 << CHARGER_CONTROL_1_EN_CHG_POS)

// REG0x18 bitfield definitions
#define CHARGER_CONTROL_3_EN_OTG_POS 6
#define CHARGER_CONTROL_3_EN_OTG_MSK (1 << CHARGER_CONTROL_3_EN_OTG_POS)

// REG0x1E bitfield definitions
#define CHARGER_STATUS_1_CHG_STAT_POS 3
#define CHARGER_STATUS_1_CHG_STAT_MSK (0x3 << CHARGER_STATUS_1_CHG_STAT_POS)
#define CHARGER_STATUS_1_VBUS_STAT_POS 0
#define CHARGER_STATUS_1_VBUS_STAT_MSK (0x7 << CHARGER_STATUS_1_VBUS_STAT_POS)

// REG0x1F bitfield definitions
#define FAULT_STATUS_0_TS_STAT_POS 0
#define FAULT_STATUS_0_TS_STAT_MSK (0x7 << FAULT_STATUS_0_TS_STAT_POS)

// REG0x38 bitfield definitions
#define PART_INFORMATION_REV_POS 0
#define PART_INFORMATION_REV_MSK (0x7 << PART_INFORMATION_REV_POS)
#define PART_INFORMATION_PN_POS 3
#define PART_INFORMATION_PN_MSK (0x7 << PART_INFORMATION_PN_POS)

// Helper macros for bitfield operations
// FIELD_GET: Extract bitfield value from register
// FIELD_PREP: Prepare bitfield value for register write
// These use __builtin_ctz (count trailing zeros) to find LSB position
// For RP2040/GCC, this is available and efficient
#define FIELD_GET(mask, reg_val) ((reg_val & mask) >> __builtin_ctz(mask))
#define FIELD_PREP(mask, val) (((val) << __builtin_ctz(mask)) & mask)

/**
 * @brief Part information from REG0x38
 */
enum bq2562x_part_number {
    BQ2562X_PART_BQ25620 = 0,
    BQ2562X_PART_BQ25622 = 1,
};

/**
 * @brief Recharge threshold offset below VREG
 */
enum bq2562x_recharge_threshold {
    BQ2562X_VRECHG_100MV = 0,
    BQ2562X_VRECHG_200MV = 1,
};

/**
 * @brief Top-off timer settings
 */
enum bq2562x_topoff_timer {
    BQ2562X_TOPOFF_DISABLED = 0,
    BQ2562X_TOPOFF_15_MINS = 1,
    BQ2562X_TOPOFF_30_MINS = 2,
    BQ2562X_TOPOFF_45_MINS = 3,
};

/**
 * @brief I2C Watchdog timer settings
 */
enum bq2562x_watchdog_setting {
    BQ2562X_WATCHDOG_DISABLE = 0,
    BQ2562X_WATCHDOG_40S = 1,
    BQ2562X_WATCHDOG_80S = 2,
    BQ2562X_WATCHDOG_160S = 3,
};

/**
 * @brief BATFET control modes
 */
enum bq2562x_batfet_ctrl {
    BQ2562X_BATFET_NORMAL = 0,
    BQ2562X_BATFET_SHUTDOWN = 1,
    BQ2562X_BATFET_SHIP_MODE = 2,
    BQ2562X_BATFET_SYS_POWER_RESET = 3,
};

/**
 * @brief Charging status from CHG_STAT bits
 */
enum bq2562x_charge_status {
    BQ2562X_CHG_STAT_NOT_CHARGING = 0,
    BQ2562X_CHG_STAT_CC_MODE = 1,      /* Trickle, Pre-charge, or Fast Charge */
    BQ2562X_CHG_STAT_CV_MODE = 2,      /* Taper Charge */
    BQ2562X_CHG_STAT_TOPOFF_ACTIVE = 3,
};

/**
 * @brief VBUS status from VBUS_STAT bits
 */
enum bq2562x_vbus_status {
    BQ2562X_VBUS_STAT_NO_INPUT = 0,
    BQ2562X_VBUS_STAT_USB_SDP = 1,
    BQ2562X_VBUS_STAT_USB_CDP = 2,
    BQ2562X_VBUS_STAT_USB_DCP = 3,
    BQ2562X_VBUS_STAT_UNKNOWN_ADAPTER = 4,
    BQ2562X_VBUS_STAT_NON_STANDARD_ADAPTER = 5,
    BQ2562X_VBUS_STAT_HVDCP = 6,
    BQ2562X_VBUS_STAT_OTG_MODE = 7,
};

/**
 * @brief NTC thermistor temperature zones
 */
enum bq2562x_ts_stat {
    BQ2562X_TS_STAT_NORMAL = 0,
    BQ2562X_TS_STAT_COLD = 1,
    BQ2562X_TS_STAT_HOT = 2,
    BQ2562X_TS_STAT_COOL = 3,
    BQ2562X_TS_STAT_WARM = 4,
    BQ2562X_TS_STAT_PRECOOL = 5,
    BQ2562X_TS_STAT_PREWARM = 6,
    BQ2562X_TS_STAT_BIAS_FAULT = 7,
};

/* --- Function Prototypes --- */

/**
 * @brief Initialize the BMS device.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_init(void);

/**
 * @brief Set the fast charge current limit.
 * @param current_ma Charge current in milliamperes (80mA to 3520mA, 80mA steps).
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_charge_current(uint16_t current_ma);

/**
 * @brief Get the fast charge current limit.
 * @param current_ma Pointer to store the charge current in milliamperes.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_charge_current(uint16_t *current_ma);

/**
 * @brief Set the charge voltage limit (VREG).
 * @param voltage_mv Charge voltage in millivolts (3500mV to 4800mV, 10mV steps).
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_charge_voltage(uint16_t voltage_mv);

/**
 * @brief Get the charge voltage limit (VREG).
 * @param voltage_mv Pointer to store the charge voltage in millivolts.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_charge_voltage(uint16_t *voltage_mv);

/**
 * @brief Set the input current limit (IINDPM).
 * @param current_ma Input current limit in milliamperes (100mA to 3200mA, 20mA steps).
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_input_current_limit(uint16_t current_ma);

/**
 * @brief Get the input current limit (IINDPM).
 * @param current_ma Pointer to store the input current limit in milliamperes.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_input_current_limit(uint16_t *current_ma);

/**
 * @brief Enable or disable charging.
 * @param enable True to enable charging, false to disable.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_charge_enable(bool enable);

/**
 * @brief Check if charging is enabled.
 * @param enabled Pointer to store the charge enable status.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_charge_enable(bool *enabled);

/**
 * @brief Set the I2C watchdog timer.
 * @param setting The desired watchdog timer setting.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_watchdog_timer(enum bq2562x_watchdog_setting setting);

/**
 * @brief Reset the I2C watchdog timer to prevent it from expiring.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_reset_watchdog_timer(void);

/**
 * @brief Enable or disable On-The-Go (OTG) boost mode.
 * @param enable True to enable OTG, false to disable.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_set_otg_enable(bool enable);

/**
 * @brief Get the charging status of the device.
 * @param status Pointer to store the current charging status.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_charge_status(enum bq2562x_charge_status *status);

/**
 * @brief Get the VBUS status (detected adapter type).
 * @param status Pointer to store the VBUS status.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_vbus_status(enum bq2562x_vbus_status *status);

/**
 * @brief Get the NTC thermistor temperature status.
 * @param status Pointer to store the TS status.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_ts_status(enum bq2562x_ts_stat *status);

/**
 * @brief Get the part information (part number and revision).
 * @param part Pointer to store the part number.
 * @param rev Pointer to store the device revision.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_part_info(enum bq2562x_part_number *part, uint8_t *rev);

/**
 * @brief Get a specific ADC reading.
 * @param reg_addr The 16-bit register address of the ADC value to read.
 * @param value Pointer to store the raw ADC value.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_adc_reading(uint8_t reg_addr, uint16_t *value);

/**
 * @brief Get the battery voltage using the ADC.
 * @param voltage_mv Pointer to store the battery voltage in millivolts.
 * @return 0 on success, negative error code on failure.
 */
int bq2562x_get_battery_voltage_oneshot(uint16_t *voltage_mv);

/**
 * @brief Puts the device into Ship Mode.
 * @return 0 on success, negative error code on failure. Returns -EBUSY if
 *         a VBUS source is present, preventing entry into ship mode.
 */
int bq2562x_enter_ship_mode(void);

/**
 * @brief Puts the device into Shutdown Mode.
 * @return 0 on success, negative error code on failure. Returns -EBUSY if
 *         a VBUS source is present, preventing entry into shutdown mode.
 */
int bq2562x_enter_shutdown_mode(void);

#endif /* BQ2562X_H_ */
