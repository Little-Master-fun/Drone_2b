#include "zf_common_debug.h"

#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"
#include "zf_device_dl1b.h"

#include "driver_vl53l1x.h"

#define DRIVER_VL53L0X_EXPECTED_MODEL_ID           (0xEEU)

#define REG_GPIO_TIO_HV_STATUS                     (0x0031U)
#define REG_RESULT_RANGE_STATUS                    (0x0089U)
#define REG_RESULT_RANGE_MM                        (0x0096U)
#define REG_FIRMWARE_SYSTEM_STATUS                 (0x00E5U)
#define REG_IDENTIFICATION_MODEL_ID                (0x010FU)

static soft_iic_info_struct driver_vl53l1x_probe_iic;

static uint8 driver_vl53l1x_initialized = 0U;
static uint8 driver_vl53l1x_model_id = 0U;
static uint8 driver_vl53l1x_boot_state = 0U;
static uint8 driver_vl53l1x_detected_type = DRIVER_TOF_SENSOR_UNKNOWN;
static uint8 driver_vl53l1x_detect_status = DRIVER_SENSOR_DETECT_UNKNOWN;

static uint8 driver_vl53l1x_read_reg8 (uint16 reg, uint8 *value)
{
    uint8 write_buf[2];

    if (0 == value)
    {
        return 1U;
    }

    write_buf[0] = (uint8)((reg >> 8) & 0xFFU);
    write_buf[1] = (uint8)(reg & 0xFFU);
    soft_iic_transfer_8bit_array(&driver_vl53l1x_probe_iic, write_buf, 2U, value, 1U);
    return 0U;
}

static void driver_vl53l1x_probe_bus_init (void)
{
    soft_iic_init(&driver_vl53l1x_probe_iic,
                  DRIVER_VL53L1X_I2C_ADDR,
                  DRIVER_VL53L1X_I2C_DELAY,
                  DRIVER_VL53L1X_SCL_PIN,
                  DRIVER_VL53L1X_SDA_PIN);
}

static void driver_vl53l1x_reset_sensor (void)
{
    gpio_init(DRIVER_VL53L1X_XS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    system_delay_ms(50);
    gpio_low(DRIVER_VL53L1X_XS_PIN);
    system_delay_ms(10);
    gpio_high(DRIVER_VL53L1X_XS_PIN);
    system_delay_ms(50);
}

static void driver_vl53l1x_refresh_probe_status (void)
{
    driver_vl53l1x_read_reg8(REG_IDENTIFICATION_MODEL_ID, &driver_vl53l1x_model_id);
    driver_vl53l1x_read_reg8(REG_FIRMWARE_SYSTEM_STATUS, &driver_vl53l1x_boot_state);
    driver_vl53l1x_detected_type = DRIVER_TOF_SENSOR_UNKNOWN;

    if (driver_vl53l1x_model_id == DRIVER_VL53L1X_EXPECTED_MODEL_ID)
    {
        driver_vl53l1x_detected_type = DRIVER_TOF_SENSOR_VL53L1X;
        driver_vl53l1x_detect_status = DRIVER_SENSOR_DETECT_OK;
        return;
    }

    if (driver_vl53l1x_model_id == DRIVER_VL53L0X_EXPECTED_MODEL_ID)
    {
        driver_vl53l1x_detected_type = DRIVER_TOF_SENSOR_VL53L0X;
        driver_vl53l1x_detect_status = DRIVER_SENSOR_DETECT_CHIP_MISMATCH;
        return;
    }

    if (((driver_vl53l1x_model_id == 0x00U) || (driver_vl53l1x_model_id == 0xFFU)) &&
        ((driver_vl53l1x_boot_state == 0x00U) || (driver_vl53l1x_boot_state == 0xFFU)))
    {
        driver_vl53l1x_detect_status = DRIVER_SENSOR_DETECT_WIRING_FAULT;
        return;
    }

    driver_vl53l1x_detect_status = DRIVER_SENSOR_DETECT_CHIP_MISMATCH;
}

uint8 driver_vl53l1x_init (void)
{
    driver_vl53l1x_initialized = 0U;
    driver_vl53l1x_model_id = 0U;
    driver_vl53l1x_boot_state = 0U;
    driver_vl53l1x_detected_type = DRIVER_TOF_SENSOR_UNKNOWN;
    driver_vl53l1x_detect_status = DRIVER_SENSOR_DETECT_UNKNOWN;

    driver_vl53l1x_probe_bus_init();
    driver_vl53l1x_reset_sensor();
    driver_vl53l1x_refresh_probe_status();

    if (driver_vl53l1x_detect_status == DRIVER_SENSOR_DETECT_WIRING_FAULT)
    {
        return 1U;
    }

    if ((driver_vl53l1x_detect_status == DRIVER_SENSOR_DETECT_CHIP_MISMATCH) ||
        (driver_vl53l1x_model_id != DRIVER_VL53L1X_EXPECTED_MODEL_ID))
    {
        return 2U;
    }

    if (0U != dl1b_init())
    {
        driver_vl53l1x_detect_status = DRIVER_SENSOR_DETECT_INIT_FAILED;
        return 3U;
    }

    driver_vl53l1x_refresh_probe_status();
    driver_vl53l1x_initialized = 1U;
    driver_vl53l1x_detect_status = DRIVER_SENSOR_DETECT_OK;
    driver_vl53l1x_detected_type = DRIVER_TOF_SENSOR_VL53L1X;
    return 0U;
}

uint8 driver_vl53l1x_read (driver_vl53l1x_data_struct *data)
{
    if (0 == data)
    {
        return 1U;
    }

    data->data_ready = 0U;
    data->range_status = 255U;

    if (!driver_vl53l1x_initialized)
    {
        return 2U;
    }

    dl1b_get_distance();
    data->distance_mm = dl1b_distance_mm;

    if (!dl1b_finsh_flag)
    {
        return 3U;
    }

    data->range_status = 0U;
    data->data_ready = 1U;
    return 0U;
}

uint8 driver_vl53l1x_is_ready (void)
{
    return driver_vl53l1x_initialized;
}

driver_vl53l1x_status_struct driver_vl53l1x_get_status (void)
{
    driver_vl53l1x_status_struct s = {0U, 0U, 0U, 0U, 0U, 0U, 0U};

    s.initialized = driver_vl53l1x_initialized;
    s.i2c_addr = DRIVER_VL53L1X_I2C_ADDR;
    s.detect_status = driver_vl53l1x_detect_status;
    s.detected_type = driver_vl53l1x_detected_type;
    s.expected_model_id = DRIVER_VL53L1X_EXPECTED_MODEL_ID;
    s.model_id = driver_vl53l1x_model_id;
    s.boot_state = driver_vl53l1x_boot_state;
    return s;
}

uint8 driver_vl53l1x_start_ranging (void)
{
    return driver_vl53l1x_initialized ? 0U : 1U;
}

uint8 driver_vl53l1x_stop_ranging (void)
{
    return driver_vl53l1x_initialized ? 0U : 1U;
}
