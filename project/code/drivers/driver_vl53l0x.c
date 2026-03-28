#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "driver_vl53l0x.h"

#define DRIVER_VL53L0X_EXPECTED_MODEL_ID           (0xEEU)
#define DRIVER_VL53L1X_EXPECTED_MODEL_ID           (0xEAU)

#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW       (0x89U)
#define REG_MSRC_CONFIG_CONTROL                    (0x60U)
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN  (0x44U)
#define REG_SYSTEM_SEQUENCE_CONFIG                 (0x01U)
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET       (0x4FU)
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD    (0x4EU)
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT      (0xB6U)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0       (0xB0U)
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO           (0x0AU)
#define REG_SYSRANGE_START                         (0x00U)
#define REG_RESULT_INTERRUPT_STATUS                (0x13U)
#define REG_SYSTEM_INTERRUPT_CLEAR                 (0x0BU)
#define REG_GPIO_HV_MUX_ACTIVE_HIGH                (0x84U)
#define REG_RESULT_RANGE_STATUS                    (0x14U)
#define REG_IDENTIFICATION_MODEL_ID                (0xC0U)
#define REG_IDENTIFICATION_REVISION_ID             (0xC2U)
#define REG_VL53L1X_IDENTIFICATION_MODEL_ID        (0x010FU)

#define DRIVER_VL53L0X_TIMEOUT_MS                  (100U)

static char driver_vl53l0x_init_err[] = "vl53l0x init error.";

static soft_iic_info_struct driver_vl53l0x_iic;
static uint8 driver_vl53l0x_driver_inited = 0U;
static uint8 driver_vl53l0x_initialized = 0U;
static uint8 driver_vl53l0x_model_id = 0U;
static uint8 driver_vl53l0x_alt_model_id = 0U;
static uint8 driver_vl53l0x_detected_type = DRIVER_TOF_SENSOR_UNKNOWN;
static uint8 driver_vl53l0x_detect_status = DRIVER_SENSOR_DETECT_UNKNOWN;
static uint8 driver_vl53l0x_stop_variable = 0U;

static void driver_vl53l0x_bus_init(void)
{
    if (driver_vl53l0x_driver_inited)
    {
        return;
    }

    soft_iic_init(&driver_vl53l0x_iic,
                  DRIVER_VL53L0X_I2C_ADDR,
                  DRIVER_VL53L0X_I2C_DELAY,
                  DRIVER_VL53L0X_SCL_PIN,
                  DRIVER_VL53L0X_SDA_PIN);
    driver_vl53l0x_driver_inited = 1U;
}

static uint8 driver_vl53l0x_write_reg8(uint8 reg, uint8 value)
{
    uint8 buf[2];
    buf[0] = reg;
    buf[1] = value;
    soft_iic_write_8bit_array(&driver_vl53l0x_iic, buf, 2U);
    return 0U;
}

static uint8 driver_vl53l0x_write_reg16(uint16 reg, uint8 value)
{
    uint8 buf[3];
    buf[0] = (uint8)((reg >> 8) & 0xFFU);
    buf[1] = (uint8)(reg & 0xFFU);
    buf[2] = value;
    soft_iic_write_8bit_array(&driver_vl53l0x_iic, buf, 3U);
    return 0U;
}

static uint8 driver_vl53l0x_write_reg_multi(uint8 reg, const uint8 *buf, uint8 len)
{
    uint8 data[7];
    uint8 i = 0U;

    if ((0 == buf) || (0U == len) || (len > 6U))
    {
        return 1U;
    }

    data[0] = reg;
    for (i = 0U; i < len; ++i)
    {
        data[1U + i] = buf[i];
    }

    soft_iic_write_8bit_array(&driver_vl53l0x_iic, data, (uint32)(len + 1U));
    return 0U;
}

static uint8 driver_vl53l0x_read_reg8(uint8 reg, uint8 *value)
{
    if (0 == value)
    {
        return 1U;
    }

    *value = soft_iic_read_8bit_register(&driver_vl53l0x_iic, reg);
    return 0U;
}

static uint8 driver_vl53l0x_read_reg16be(uint8 reg, uint16 *value)
{
    uint8 data[2];

    if (0 == value)
    {
        return 1U;
    }

    soft_iic_read_8bit_registers(&driver_vl53l0x_iic, reg, data, 2U);
    *value = (uint16)(((uint16)data[0] << 8) | data[1]);
    return 0U;
}

static uint8 driver_vl53l0x_read_reg16_addr8(uint16 reg, uint8 *value)
{
    uint8 write_buf[2];

    if (0 == value)
    {
        return 1U;
    }

    write_buf[0] = (uint8)((reg >> 8) & 0xFFU);
    write_buf[1] = (uint8)(reg & 0xFFU);
    soft_iic_transfer_8bit_array(&driver_vl53l0x_iic, write_buf, 2U, value, 1U);
    return 0U;
}

static uint8 driver_vl53l0x_read_reg_multi(uint8 reg, uint8 *buf, uint8 len)
{
    if ((0 == buf) || (0U == len))
    {
        return 1U;
    }

    soft_iic_read_8bit_registers(&driver_vl53l0x_iic, reg, buf, len);
    return 0U;
}

static void driver_vl53l0x_refresh_probe_status(void)
{
    uint8 revision_id = 0U;

    (void)driver_vl53l0x_read_reg8(REG_IDENTIFICATION_MODEL_ID, &driver_vl53l0x_model_id);
    (void)driver_vl53l0x_read_reg8(REG_IDENTIFICATION_REVISION_ID, &revision_id);
    (void)driver_vl53l0x_read_reg16_addr8(REG_VL53L1X_IDENTIFICATION_MODEL_ID, &driver_vl53l0x_alt_model_id);
    driver_vl53l0x_detected_type = DRIVER_TOF_SENSOR_UNKNOWN;

    if (driver_vl53l0x_model_id == DRIVER_VL53L0X_EXPECTED_MODEL_ID)
    {
        driver_vl53l0x_detected_type = DRIVER_TOF_SENSOR_VL53L0X;
        driver_vl53l0x_detect_status = DRIVER_SENSOR_DETECT_OK;
        return;
    }

    if (driver_vl53l0x_alt_model_id == DRIVER_VL53L1X_EXPECTED_MODEL_ID)
    {
        driver_vl53l0x_detected_type = DRIVER_TOF_SENSOR_VL53L1X;
        driver_vl53l0x_detect_status = DRIVER_SENSOR_DETECT_CHIP_MISMATCH;
        return;
    }

    if (((driver_vl53l0x_model_id == 0x00U) || (driver_vl53l0x_model_id == 0xFFU)) &&
        ((revision_id == 0x00U) || (revision_id == 0xFFU)) &&
        ((driver_vl53l0x_alt_model_id == 0x00U) || (driver_vl53l0x_alt_model_id == 0xFFU)))
    {
        driver_vl53l0x_detect_status = DRIVER_SENSOR_DETECT_WIRING_FAULT;
        return;
    }

    driver_vl53l0x_detect_status = DRIVER_SENSOR_DETECT_CHIP_MISMATCH;
}

static uint8 driver_vl53l0x_sensor_tuning(void)
{
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x00U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x09U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x10U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x11U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x24U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x25U, 0xFFU);
    (void)driver_vl53l0x_write_reg8(0x75U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x4EU, 0x2CU);
    (void)driver_vl53l0x_write_reg8(0x48U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x30U, 0x20U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x30U, 0x09U);
    (void)driver_vl53l0x_write_reg8(0x54U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x31U, 0x04U);
    (void)driver_vl53l0x_write_reg8(0x32U, 0x03U);
    (void)driver_vl53l0x_write_reg8(0x40U, 0x83U);
    (void)driver_vl53l0x_write_reg8(0x46U, 0x25U);
    (void)driver_vl53l0x_write_reg8(0x60U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x27U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x50U, 0x06U);
    (void)driver_vl53l0x_write_reg8(0x51U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x52U, 0x96U);
    (void)driver_vl53l0x_write_reg8(0x56U, 0x08U);
    (void)driver_vl53l0x_write_reg8(0x57U, 0x30U);
    (void)driver_vl53l0x_write_reg8(0x61U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x62U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x64U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x65U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x66U, 0xA0U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x22U, 0x32U);
    (void)driver_vl53l0x_write_reg8(0x47U, 0x14U);
    (void)driver_vl53l0x_write_reg8(0x49U, 0xFFU);
    (void)driver_vl53l0x_write_reg8(0x4AU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x7AU, 0x0AU);
    (void)driver_vl53l0x_write_reg8(0x7BU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x78U, 0x21U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x23U, 0x34U);
    (void)driver_vl53l0x_write_reg8(0x42U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x44U, 0xFFU);
    (void)driver_vl53l0x_write_reg8(0x45U, 0x26U);
    (void)driver_vl53l0x_write_reg8(0x46U, 0x05U);
    (void)driver_vl53l0x_write_reg8(0x40U, 0x40U);
    (void)driver_vl53l0x_write_reg8(0x0EU, 0x06U);
    (void)driver_vl53l0x_write_reg8(0x20U, 0x1AU);
    (void)driver_vl53l0x_write_reg8(0x43U, 0x40U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x34U, 0x03U);
    (void)driver_vl53l0x_write_reg8(0x35U, 0x44U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x31U, 0x04U);
    (void)driver_vl53l0x_write_reg8(0x4BU, 0x09U);
    (void)driver_vl53l0x_write_reg8(0x4CU, 0x05U);
    (void)driver_vl53l0x_write_reg8(0x4DU, 0x04U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x44U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x45U, 0x20U);
    (void)driver_vl53l0x_write_reg8(0x47U, 0x08U);
    (void)driver_vl53l0x_write_reg8(0x48U, 0x28U);
    (void)driver_vl53l0x_write_reg8(0x67U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x70U, 0x04U);
    (void)driver_vl53l0x_write_reg8(0x71U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x72U, 0xFEU);
    (void)driver_vl53l0x_write_reg8(0x76U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x77U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x0DU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x80U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x01U, 0xF8U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x8EU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x00U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x80U, 0x00U);
    return 0U;
}

static uint8 driver_vl53l0x_single_ref_calibration(uint8 vhv_init_byte)
{
    uint8 status = 0U;
    uint16 wait_ms = 0U;

    (void)driver_vl53l0x_write_reg8(REG_SYSRANGE_START, (uint8)(vhv_init_byte | 0x01U));

    while (wait_ms < DRIVER_VL53L0X_TIMEOUT_MS)
    {
        (void)driver_vl53l0x_read_reg8(REG_RESULT_INTERRUPT_STATUS, &status);

        if ((status & 0x07U) != 0U)
        {
            (void)driver_vl53l0x_write_reg8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);
            (void)driver_vl53l0x_write_reg8(REG_SYSRANGE_START, 0x00U);
            return 0U;
        }

        system_delay_ms(1U);
        ++wait_ms;
    }

    return 1U;
}

static uint8 driver_vl53l0x_spad_calculations(void)
{
    uint8 val = 0U;
    uint8 spad_count = 0U;
    uint8 ref_spad_map[6] = {0};
    uint8 i = 0U;
    uint8 first_spad_to_enable = 0U;
    uint8 spads_enabled = 0U;
    uint16 wait_ms = 0U;
    uint8 spad_type_is_aperture = 0U;

    (void)driver_vl53l0x_write_reg8(0x80U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x00U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x06U);
    (void)driver_vl53l0x_read_reg8(0x83U, &val);
    (void)driver_vl53l0x_write_reg8(0x83U, (uint8)(val | 0x04U));
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x07U);
    (void)driver_vl53l0x_write_reg8(0x81U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x80U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x94U, 0x6BU);
    (void)driver_vl53l0x_write_reg8(0x83U, 0x00U);

    while (wait_ms < DRIVER_VL53L0X_TIMEOUT_MS)
    {
        (void)driver_vl53l0x_read_reg8(0x83U, &val);
        if (val != 0x00U)
        {
            break;
        }

        system_delay_ms(1U);
        ++wait_ms;
    }

    if (val == 0x00U)
    {
        return 1U;
    }

    (void)driver_vl53l0x_write_reg8(0x83U, 0x01U);
    (void)driver_vl53l0x_read_reg8(0x92U, &val);

    spad_count = (uint8)(val & 0x7FU);
    spad_type_is_aperture = (uint8)((val >> 7) & 0x01U);

    (void)driver_vl53l0x_write_reg8(0x81U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x06U);
    (void)driver_vl53l0x_read_reg8(0x83U, &val);
    (void)driver_vl53l0x_write_reg8(0x83U, (uint8)(val & (uint8)(~0x04U)));
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x00U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x80U, 0x00U);

    (void)driver_vl53l0x_read_reg_multi(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6U);

    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00U);
    (void)driver_vl53l0x_write_reg8(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2CU);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4U);

    first_spad_to_enable = spad_type_is_aperture ? 12U : 0U;

    for (i = 0U; i < 48U; ++i)
    {
        if ((i < first_spad_to_enable) || (spads_enabled == spad_count))
        {
            ref_spad_map[i / 8U] &= (uint8)(~(1U << (i % 8U)));
        }
        else if (((ref_spad_map[i / 8U] >> (i % 8U)) & 0x01U) != 0U)
        {
            ++spads_enabled;
        }
    }

    (void)driver_vl53l0x_write_reg_multi(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6U);

    if (driver_vl53l0x_sensor_tuning() != 0U)
    {
        return 2U;
    }

    (void)driver_vl53l0x_write_reg8(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04U);
    (void)driver_vl53l0x_read_reg8(REG_GPIO_HV_MUX_ACTIVE_HIGH, &val);
    (void)driver_vl53l0x_write_reg8(REG_GPIO_HV_MUX_ACTIVE_HIGH, (uint8)(val & (uint8)(~0x10U)));
    (void)driver_vl53l0x_write_reg8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);
    (void)driver_vl53l0x_write_reg8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8U);
    (void)driver_vl53l0x_write_reg8(REG_SYSTEM_SEQUENCE_CONFIG, 0x01U);

    if (driver_vl53l0x_single_ref_calibration(0x40U) != 0U)
    {
        return 3U;
    }

    (void)driver_vl53l0x_write_reg8(REG_SYSTEM_SEQUENCE_CONFIG, 0x02U);

    if (driver_vl53l0x_single_ref_calibration(0x00U) != 0U)
    {
        return 4U;
    }

    (void)driver_vl53l0x_write_reg8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8U);
    return 0U;
}

static uint8 driver_vl53l0x_sensor_init(void)
{
    uint8 val = 0U;
    uint8 rate_limit_split[2];

    (void)driver_vl53l0x_read_reg8(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW, &val);
    (void)driver_vl53l0x_write_reg8(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HW, (uint8)(val | 0x01U));

    (void)driver_vl53l0x_write_reg8(0x88U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x80U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x00U, 0x00U);
    (void)driver_vl53l0x_read_reg8(0x91U, &val);
    (void)driver_vl53l0x_write_reg8(0x00U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x80U, 0x00U);
    driver_vl53l0x_stop_variable = val;

    (void)driver_vl53l0x_read_reg8(REG_MSRC_CONFIG_CONTROL, &val);
    (void)driver_vl53l0x_write_reg8(REG_MSRC_CONFIG_CONTROL, (uint8)(val | 0x12U));

    rate_limit_split[0] = 0x19U;
    rate_limit_split[1] = 0x99U;
    (void)driver_vl53l0x_write_reg_multi(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN, rate_limit_split, 2U);
    (void)driver_vl53l0x_write_reg8(REG_SYSTEM_SEQUENCE_CONFIG, 0xFFU);

    return driver_vl53l0x_spad_calculations();
}

static uint8 driver_vl53l0x_perform_single_measurement(driver_vl53l0x_data_struct *data)
{
    uint8 system_start = 0U;
    uint8 interrupt_status = 0U;
    uint16 distance_mm = 0U;
    uint16 wait_ms = 0U;

    if (0 == data)
    {
        return 1U;
    }

    (void)driver_vl53l0x_write_reg8(0x80U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x01U);
    (void)driver_vl53l0x_write_reg8(0x00U, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x91U, driver_vl53l0x_stop_variable);
    (void)driver_vl53l0x_write_reg8(0x00U, 0x01U);
    (void)driver_vl53l0x_write_reg8(0xFFU, 0x00U);
    (void)driver_vl53l0x_write_reg8(0x80U, 0x00U);
    (void)driver_vl53l0x_write_reg8(REG_SYSRANGE_START, 0x01U);

    while (wait_ms < DRIVER_VL53L0X_TIMEOUT_MS)
    {
        (void)driver_vl53l0x_read_reg8(REG_SYSRANGE_START, &system_start);
        if ((system_start & 0x01U) == 0U)
        {
            break;
        }

        system_delay_ms(1U);
        ++wait_ms;
    }

    if ((system_start & 0x01U) != 0U)
    {
        return 2U;
    }

    wait_ms = 0U;
    while (wait_ms < DRIVER_VL53L0X_TIMEOUT_MS)
    {
        (void)driver_vl53l0x_read_reg8(REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
        if ((interrupt_status & 0x07U) != 0U)
        {
            break;
        }

        system_delay_ms(1U);
        ++wait_ms;
    }

    if ((interrupt_status & 0x07U) == 0U)
    {
        return 3U;
    }

    (void)driver_vl53l0x_read_reg16be((uint8)(REG_RESULT_RANGE_STATUS + 10U), &distance_mm);
    (void)driver_vl53l0x_read_reg8(REG_RESULT_RANGE_STATUS, &data->range_status);
    (void)driver_vl53l0x_write_reg8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);

    data->distance_mm = distance_mm;
    data->data_ready = 1U;
    return 0U;
}

uint8 driver_vl53l0x_init(void)
{
    driver_vl53l0x_initialized = 0U;
    driver_vl53l0x_stop_variable = 0U;
    driver_vl53l0x_model_id = 0U;
    driver_vl53l0x_alt_model_id = 0U;
    driver_vl53l0x_detected_type = DRIVER_TOF_SENSOR_UNKNOWN;
    driver_vl53l0x_detect_status = DRIVER_SENSOR_DETECT_UNKNOWN;

    driver_vl53l0x_bus_init();
    driver_vl53l0x_refresh_probe_status();

    if (driver_vl53l0x_detect_status == DRIVER_SENSOR_DETECT_WIRING_FAULT)
    {
        return 1U;
    }

    if ((driver_vl53l0x_detect_status == DRIVER_SENSOR_DETECT_CHIP_MISMATCH) ||
        (driver_vl53l0x_model_id != DRIVER_VL53L0X_EXPECTED_MODEL_ID))
    {
        return 2U;
    }

    if (driver_vl53l0x_sensor_init() != 0U)
    {
        driver_vl53l0x_detect_status = DRIVER_SENSOR_DETECT_INIT_FAILED;
        zf_log(0, driver_vl53l0x_init_err);
        return 3U;
    }

    driver_vl53l0x_initialized = 1U;
    driver_vl53l0x_detect_status = DRIVER_SENSOR_DETECT_OK;
    driver_vl53l0x_detected_type = DRIVER_TOF_SENSOR_VL53L0X;
    return 0U;
}

uint8 driver_vl53l0x_read(driver_vl53l0x_data_struct *data)
{
    if (0 == data)
    {
        return 1U;
    }

    data->data_ready = 0U;
    data->range_status = 255U;

    if (!driver_vl53l0x_initialized)
    {
        return 2U;
    }

    return driver_vl53l0x_perform_single_measurement(data);
}

uint8 driver_vl53l0x_is_ready(void)
{
    return driver_vl53l0x_initialized;
}

driver_vl53l0x_status_struct driver_vl53l0x_get_status(void)
{
    driver_vl53l0x_status_struct s = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
    s.initialized = driver_vl53l0x_initialized;
    s.i2c_addr = DRIVER_VL53L0X_I2C_ADDR;
    s.detect_status = driver_vl53l0x_detect_status;
    s.detected_type = driver_vl53l0x_detected_type;
    s.expected_model_id = DRIVER_VL53L0X_EXPECTED_MODEL_ID;
    s.model_id = driver_vl53l0x_model_id;
    s.alt_model_id = driver_vl53l0x_alt_model_id;
    return s;
}

uint8 driver_vl53l0x_start_ranging(void)
{
    return driver_vl53l0x_initialized ? 0U : 1U;
}

uint8 driver_vl53l0x_stop_ranging(void)
{
    (void)driver_vl53l0x_write_reg8(REG_SYSRANGE_START, 0x00U);
    return 0U;
}
