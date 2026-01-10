/*
 * BQ2562X Battery Charger Driver for Arduino/RP2040
 * Ported from Zephyr driver
 */

#include "bq2562x.h"

/* --- I2C Helper Functions --- */

static int bq2562x_read_reg(uint8_t reg, uint8_t *val)
{
    Wire.beginTransmission(BQ2562X_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return -1;  // I2C error
    }
    if (Wire.requestFrom(BQ2562X_I2C_ADDR, (uint8_t)1) != 1) {
        return -1;  // Read error
    }
    *val = Wire.read();
    return 0;
}

static int bq2562x_write_reg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(BQ2562X_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    if (Wire.endTransmission() != 0) {
        return -1;  // I2C error
    }
    return 0;
}

static int bq2562x_update_reg(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t reg_val;
    int ret = bq2562x_read_reg(reg, &reg_val);
    if (ret != 0) {
        return ret;
    }
    reg_val = (reg_val & ~mask) | (val & mask);
    return bq2562x_write_reg(reg, reg_val);
}

static int bq2562x_read_reg16(uint8_t reg, uint16_t *val)
{
    Wire.beginTransmission(BQ2562X_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return -1;
    }
    if (Wire.requestFrom(BQ2562X_I2C_ADDR, (uint8_t)2) != 2) {
        return -1;
    }
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    *val = ((uint16_t)msb << 8) | lsb;
    return 0;
}

static int bq2562x_write_reg16(uint8_t reg, uint16_t val)
{
    Wire.beginTransmission(BQ2562X_I2C_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(val & 0xFF));        // LSB
    Wire.write((uint8_t)((val >> 8) & 0xFF)); // MSB
    if (Wire.endTransmission() != 0) {
        return -1;
    }
    return 0;
}

/* --- API Implementation --- */

int bq2562x_init(void)
{
    enum bq2562x_part_number part;
    uint8_t rev;
    int ret;

    // Check if I2C device responds
    Wire.beginTransmission(BQ2562X_I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        return -1;  // Device not found
    }

    ret = bq2562x_get_part_info(&part, &rev);
    if (ret < 0) {
        return ret;
    }

    /* Reset watchdog timer to start in a known state */
    ret = bq2562x_reset_watchdog_timer();
    if (ret < 0) {
        return ret;
    }

    return 0;
}

int bq2562x_set_charge_current(uint16_t current_ma)
{
    if (current_ma < MIN_CURRENT_CHARGE_LIMIT || current_ma > MAX_CURRENT_CHARGE_LIMIT) {
        return -1;  // Invalid parameter
    }
    uint16_t reg_val = MA_TO_CHARGE_CURRENT_LIMIT(current_ma);
    return bq2562x_write_reg16(BQ2562X_REG_CHARGE_CURRENT_LIMIT, reg_val);
}

int bq2562x_get_charge_current(uint16_t *current_ma)
{
    uint16_t reg_val;
    int ret = bq2562x_read_reg16(BQ2562X_REG_CHARGE_CURRENT_LIMIT, &reg_val);
    if (ret == 0) {
        *current_ma = CHARGE_CURRENT_LIMIT_TO_MA(reg_val);
    }
    return ret;
}

int bq2562x_set_charge_voltage(uint16_t voltage_mv)
{
    if (voltage_mv < MIN_CHARGE_VOLTAGE_LIMIT || voltage_mv > MAX_CHARGE_VOLTAGE_LIMIT) {
        return -1;
    }
    uint16_t reg_val = MV_TO_CHARGE_VOLTAGE_LIMIT(voltage_mv);
    return bq2562x_write_reg16(BQ2562X_REG_CHARGE_VOLTAGE_LIMIT, reg_val);
}

int bq2562x_get_charge_voltage(uint16_t *voltage_mv)
{
    uint16_t reg_val;
    int ret = bq2562x_read_reg16(BQ2562X_REG_CHARGE_VOLTAGE_LIMIT, &reg_val);
    if (ret == 0) {
        *voltage_mv = CHARGE_VOLTAGE_LIMIT_TO_MV(reg_val);
    }
    return ret;
}

int bq2562x_set_input_current_limit(uint16_t current_ma)
{
    if (current_ma < MIN_INPUT_CURRENT_LIMIT || current_ma > MAX_INPUT_CURRENT_LIMIT) {
        return -1;
    }
    uint16_t reg_val = MA_TO_INPUT_CURRENT_LIMIT(current_ma);
    return bq2562x_write_reg16(BQ2562X_REG_INPUT_CURRENT_LIMIT, reg_val);
}

int bq2562x_get_input_current_limit(uint16_t *current_ma)
{
    uint16_t reg_val;
    int ret = bq2562x_read_reg16(BQ2562X_REG_INPUT_CURRENT_LIMIT, &reg_val);
    if (ret == 0) {
        *current_ma = INPUT_CURRENT_LIMIT_TO_MA(reg_val);
    }
    return ret;
}

int bq2562x_set_charge_enable(bool enable)
{
    uint8_t val = enable ? FIELD_PREP(CHARGER_CONTROL_1_EN_CHG_MSK, 1) : 0;
    return bq2562x_update_reg(BQ2562X_REG_CHARGER_CONTROL_1,
                   CHARGER_CONTROL_1_EN_CHG_MSK, val);
}

int bq2562x_get_charge_enable(bool *enabled)
{
    uint8_t reg_val;
    int ret = bq2562x_read_reg(BQ2562X_REG_CHARGER_CONTROL_1, &reg_val);
    if (ret == 0) {
        *enabled = (bool)FIELD_GET(CHARGER_CONTROL_1_EN_CHG_MSK, reg_val);
    }
    return ret;
}

int bq2562x_set_watchdog_timer(enum bq2562x_watchdog_setting setting)
{
    uint8_t val = FIELD_PREP(CHARGER_CONTROL_1_WATCHDOG_MSK, setting);
    return bq2562x_update_reg(BQ2562X_REG_CHARGER_CONTROL_1,
                   CHARGER_CONTROL_1_WATCHDOG_MSK, val);
}

int bq2562x_reset_watchdog_timer(void)
{
    return bq2562x_update_reg(BQ2562X_REG_CHARGER_CONTROL_1,
                   CHARGER_CONTROL_1_WD_RST_MSK,
                   FIELD_PREP(CHARGER_CONTROL_1_WD_RST_MSK, 1));
}

int bq2562x_set_otg_enable(bool enable)
{
    uint8_t val = enable ? FIELD_PREP(CHARGER_CONTROL_3_EN_OTG_MSK, 1) : 0;
    return bq2562x_update_reg(BQ2562X_REG_CHARGER_CONTROL_3,
                   CHARGER_CONTROL_3_EN_OTG_MSK, val);
}

int bq2562x_get_charge_status(enum bq2562x_charge_status *status)
{
    uint8_t reg_val;
    int ret = bq2562x_read_reg(BQ2562X_REG_CHARGER_STATUS_1, &reg_val);
    if (ret == 0) {
        *status = (enum bq2562x_charge_status)FIELD_GET(CHARGER_STATUS_1_CHG_STAT_MSK, reg_val);
    }
    return ret;
}

int bq2562x_get_vbus_status(enum bq2562x_vbus_status *status)
{
    uint8_t reg_val;
    int ret = bq2562x_read_reg(BQ2562X_REG_CHARGER_STATUS_1, &reg_val);
    if (ret == 0) {
        *status = (enum bq2562x_vbus_status)FIELD_GET(CHARGER_STATUS_1_VBUS_STAT_MSK, reg_val);
    }
    return ret;
}

int bq2562x_get_ts_status(enum bq2562x_ts_stat *status)
{
    uint8_t reg_val;
    int ret = bq2562x_read_reg(BQ2562X_REG_FAULT_STATUS_0, &reg_val);
    if (ret == 0) {
        *status = (enum bq2562x_ts_stat)FIELD_GET(FAULT_STATUS_0_TS_STAT_MSK, reg_val);
    }
    return ret;
}

int bq2562x_get_part_info(enum bq2562x_part_number *part, uint8_t *rev)
{
    uint8_t reg_val;
    int ret = bq2562x_read_reg(BQ2562X_REG_PART_INFORMATION, &reg_val);
    if (ret == 0) {
        *part = (enum bq2562x_part_number)FIELD_GET(PART_INFORMATION_PN_MSK, reg_val);
        *rev = FIELD_GET(PART_INFORMATION_REV_MSK, reg_val);
    }
    return ret;
}

int bq2562x_get_adc_reading(uint8_t reg_addr, uint16_t *value)
{
    return bq2562x_read_reg16(reg_addr, value);
}

int bq2562x_get_battery_voltage_oneshot(uint16_t *voltage_mv)
{
    int ret;
    uint16_t raw_adc_val;

    // Step 1: Read the current (potentially stale) ADC value first.
    ret = bq2562x_read_reg16(BQ2562X_REG_VBAT_ADC, &raw_adc_val);
    if (ret != 0) {
        return ret;
    }

    // Step 2: Start a one-shot conversion.
    uint8_t adc_ctrl_val = FIELD_PREP(ADC_CONTROL_EN_MSK, 1) |
               FIELD_PREP(ADC_CONTROL_RATE_MSK, 1);
    ret = bq2562x_update_reg(BQ2562X_REG_ADC_CONTROL,
                 ADC_CONTROL_EN_MSK | ADC_CONTROL_RATE_MSK,
                 adc_ctrl_val);
    if (ret != 0) {
        return ret;
    }

    // Step 3: Poll for completion.
    uint8_t adc_stat = 0;
    for (int i = 0; i < BQ2562X_ADC_CONVERSION_TIMEOUT_MS; i++) {
        ret = bq2562x_read_reg(BQ2562X_REG_CHARGER_STATUS_0, &adc_stat);
        if (ret != 0) {
            return ret;
        }
        if (FIELD_GET(ADC_DONE_STAT_MSK, adc_stat)) {
            break;
        }
        delay(1);
    }

    if (!FIELD_GET(ADC_DONE_STAT_MSK, adc_stat)) {
        return -2;  // Timeout
    }

    // Step 4: Read the fresh ADC value.
    ret = bq2562x_read_reg16(BQ2562X_REG_VBAT_ADC, &raw_adc_val);
    if (ret != 0) {
        return ret;
    }

    uint16_t shifted_val = (raw_adc_val & 0x1FFE) >> 1;
    *voltage_mv = (uint16_t)(shifted_val * BQ2562X_VBAT_ADC_LSB_MV);

    return 0;
}

int bq2562x_enter_ship_mode(void)
{
    int ret;
    enum bq2562x_vbus_status vbus_stat;

    // Step 1: Check if a VBUS source is present.
    ret = bq2562x_get_vbus_status(&vbus_stat);
    if (ret) {
        return ret;
    }

    if (vbus_stat != BQ2562X_VBUS_STAT_NO_INPUT) {
        return -3;  // EBUSY equivalent
    }

    // Step 2: Write the Ship Mode command to the BATFET_CTRL bits.
    uint8_t val = FIELD_PREP(BATFET_CTRL_MSK, BQ2562X_BATFET_SHIP_MODE);
    ret = bq2562x_update_reg(BQ2562X_REG_CHARGER_CONTROL_3, BATFET_CTRL_MSK, val);

    return ret;
}

int bq2562x_enter_shutdown_mode(void)
{
    int ret;
    enum bq2562x_vbus_status vbus_stat;

    // Step 1: Check if a VBUS source is present.
    ret = bq2562x_get_vbus_status(&vbus_stat);
    if (ret) {
        return ret;
    }

    if (vbus_stat != BQ2562X_VBUS_STAT_NO_INPUT) {
        return -3;  // EBUSY equivalent - VBUS present
    }

    // Step 2: Write the Shutdown Mode command to the BATFET_CTRL bits.
    uint8_t val = FIELD_PREP(BATFET_CTRL_MSK, BQ2562X_BATFET_SHUTDOWN);
    ret = bq2562x_update_reg(BQ2562X_REG_CHARGER_CONTROL_3, BATFET_CTRL_MSK, val);

    return ret;
}
