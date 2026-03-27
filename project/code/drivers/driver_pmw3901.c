#include "zf_common_debug.h"
#include "zf_common_function.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include "driver_pmw3901.h"

#define PMW3901_CHIP_ID                  (0x49U)
static uint8 driver_pmw3901_spi_inited = 0U;
static uint8 driver_pmw3901_tx_buf[2];
static uint8 driver_pmw3901_rx_buf[2];
static char driver_pmw3901_chip_id_err[] = "pmw3901 chip id error.";

static driver_pmw3901_status_struct driver_pmw3901_status = {
    0U, DRIVER_SENSOR_DETECT_UNKNOWN, PMW3901_CHIP_ID, 0U, 0U
};

static const uint8 driver_pmw3901_init_seq[][2] = {
    {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05},
    {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14}, {0x5B, 0x32}, {0x5F, 0x34},
    {0x7B, 0x08}, {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F},
    {0x7F, 0x08}, {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
    {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77}, {0x60, 0x78},
    {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50}, {0x7F, 0x0A}, {0x45, 0x60},
    {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x21}, {0x75, 0x1F},
    {0x4A, 0x78}, {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF},
    {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x67}, {0x66, 0x08}, {0x63, 0x70},
    {0x7F, 0x15}, {0x48, 0x48}, {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14},
    {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10},
    {0x5B, 0x02}, {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00}
};

static uint8 driver_pmw3901_classify_probe(uint8 chip_id, uint8 chip_id_inv)
{
    if (chip_id == PMW3901_CHIP_ID)
    {
        return DRIVER_SENSOR_DETECT_OK;
    }

    if (((chip_id == 0x00U) || (chip_id == 0xFFU)) &&
        ((chip_id_inv == 0x00U) || (chip_id_inv == 0xFFU)))
    {
        return DRIVER_SENSOR_DETECT_WIRING_FAULT;
    }

    return DRIVER_SENSOR_DETECT_CHIP_MISMATCH;
}

static void driver_pmw3901_adjust_trim(uint8 *c1, uint8 *c2)
{
    if ((0 == c1) || (0 == c2))
    {
        return;
    }

    if (*c1 <= 28U)
    {
        *c1 = (uint8)(*c1 + 14U);
    }
    else
    {
        *c1 = (uint8)(*c1 + 11U);
    }

    if (*c1 > 0x3FU)
    {
        *c1 = 0x3FU;
    }

    *c2 = (uint8)(((uint16)(*c2) * 45U) / 100U);
}

static void driver_pmw3901_spi_init(void)
{
    if (driver_pmw3901_spi_inited)
    {
        return;
    }

    spi_init(DRIVER_PMW3901_SPI_INDEX,
             SPI_MODE3,
             DRIVER_PMW3901_SPI_BAUD,
             DRIVER_PMW3901_SPI_CLK_PIN,
             DRIVER_PMW3901_SPI_MOSI_PIN,
             DRIVER_PMW3901_SPI_MISO_PIN,
             SPI_CS_NULL);

    gpio_init(DRIVER_PMW3901_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    driver_pmw3901_spi_inited = 1U;
}

static uint8 driver_pmw3901_spi_transfer(uint8 out_byte)
{
    driver_pmw3901_tx_buf[0] = out_byte;
    spi_transfer_8bit(DRIVER_PMW3901_SPI_INDEX, driver_pmw3901_tx_buf, driver_pmw3901_rx_buf, 1);
    return driver_pmw3901_rx_buf[0];
}

static void driver_pmw3901_register_write(uint8 reg, uint8 value)
{
    gpio_low(DRIVER_PMW3901_CS_PIN);
    system_delay_us(50);
    (void)driver_pmw3901_spi_transfer((uint8)(reg | 0x80U));
    (void)driver_pmw3901_spi_transfer(value);
    system_delay_us(50);
    gpio_high(DRIVER_PMW3901_CS_PIN);
    system_delay_us(200);
}

static uint8 driver_pmw3901_register_read(uint8 reg)
{
    uint8 value = 0U;

    gpio_low(DRIVER_PMW3901_CS_PIN);
    system_delay_us(50);
    (void)driver_pmw3901_spi_transfer((uint8)(reg & 0x7FU));
    system_delay_us(50);
    value = driver_pmw3901_spi_transfer(0x00U);
    system_delay_us(100);
    gpio_high(DRIVER_PMW3901_CS_PIN);
    system_delay_us(1);

    return value;
}

static void driver_pmw3901_init_registers(void)
{
    uint8 value = 0U;
    uint8 trim_c1 = 0U;
    uint8 trim_c2 = 0U;
    uint32 i = 0U;

    driver_pmw3901_register_write(0x7F, 0x00);
    driver_pmw3901_register_write(0x55, 0x01);
    driver_pmw3901_register_write(0x50, 0x07);
    driver_pmw3901_register_write(0x7F, 0x0E);
    driver_pmw3901_register_write(0x43, 0x10);

    value = driver_pmw3901_register_read(0x67);
    driver_pmw3901_register_write(0x48, (value & 0x80U) ? 0x04U : 0x02U);

    driver_pmw3901_register_write(0x7F, 0x00);
    driver_pmw3901_register_write(0x51, 0x7B);
    driver_pmw3901_register_write(0x50, 0x00);
    driver_pmw3901_register_write(0x55, 0x00);
    driver_pmw3901_register_write(0x7F, 0x0E);

    value = driver_pmw3901_register_read(0x73);
    if (0U == value)
    {
        trim_c1 = driver_pmw3901_register_read(0x70);
        trim_c2 = driver_pmw3901_register_read(0x71);
        driver_pmw3901_adjust_trim(&trim_c1, &trim_c2);

        driver_pmw3901_register_write(0x7F, 0x00);
        driver_pmw3901_register_write(0x61, 0xAD);
        driver_pmw3901_register_write(0x51, 0x70);
        driver_pmw3901_register_write(0x7F, 0x0E);
        driver_pmw3901_register_write(0x70, trim_c1);
        driver_pmw3901_register_write(0x71, trim_c2);
    }

    for (i = 0U; i < (sizeof(driver_pmw3901_init_seq) / sizeof(driver_pmw3901_init_seq[0])); ++i)
    {
        driver_pmw3901_register_write(driver_pmw3901_init_seq[i][0], driver_pmw3901_init_seq[i][1]);
    }

    system_delay_ms(10);

    driver_pmw3901_register_write(0x32, 0x44);
    driver_pmw3901_register_write(0x7F, 0x07);
    driver_pmw3901_register_write(0x40, 0x40);
    driver_pmw3901_register_write(0x7F, 0x06);
    driver_pmw3901_register_write(0x62, 0xF0);
    driver_pmw3901_register_write(0x63, 0x00);
    driver_pmw3901_register_write(0x7F, 0x0D);
    driver_pmw3901_register_write(0x48, 0xC0);
    driver_pmw3901_register_write(0x6F, 0xD5);
    driver_pmw3901_register_write(0x7F, 0x00);
    driver_pmw3901_register_write(0x5B, 0xA0);
    driver_pmw3901_register_write(0x4E, 0xA8);
    driver_pmw3901_register_write(0x5A, 0x50);
    driver_pmw3901_register_write(0x40, 0x80);
}

uint8 driver_pmw3901_init(void)
{
    uint8 chip_id = 0U;
    uint8 chip_id_inv = 0U;

    driver_pmw3901_status.initialized = 0U;
    driver_pmw3901_status.detect_status = DRIVER_SENSOR_DETECT_UNKNOWN;
    driver_pmw3901_status.expected_chip_id = PMW3901_CHIP_ID;
    driver_pmw3901_status.chip_id = 0U;
    driver_pmw3901_status.chip_id_inverse = 0U;

    driver_pmw3901_spi_init();

    gpio_high(DRIVER_PMW3901_CS_PIN);
    system_delay_ms(1);
    gpio_low(DRIVER_PMW3901_CS_PIN);
    system_delay_ms(1);
    gpio_high(DRIVER_PMW3901_CS_PIN);
    system_delay_ms(1);

    driver_pmw3901_register_write(0x3A, 0x5A);
    system_delay_ms(5);

    chip_id = driver_pmw3901_register_read(0x00);
    chip_id_inv = driver_pmw3901_register_read(0x5F);

    driver_pmw3901_status.chip_id = chip_id;
    driver_pmw3901_status.chip_id_inverse = chip_id_inv;
    driver_pmw3901_status.detect_status = driver_pmw3901_classify_probe(chip_id, chip_id_inv);

    if (PMW3901_CHIP_ID != chip_id)
    {
        debug_log_handler(0, driver_pmw3901_chip_id_err, (char *)__FILE__, __LINE__);
        return 1U;
    }

    (void)driver_pmw3901_register_read(0x02);
    (void)driver_pmw3901_register_read(0x03);
    (void)driver_pmw3901_register_read(0x04);
    (void)driver_pmw3901_register_read(0x05);
    (void)driver_pmw3901_register_read(0x06);
    system_delay_ms(1);

    driver_pmw3901_init_registers();

    driver_pmw3901_status.initialized = 1U;
    driver_pmw3901_status.detect_status = DRIVER_SENSOR_DETECT_OK;
    return 0U;
}

uint8 driver_pmw3901_read_motion(driver_pmw3901_motion_struct *motion)
{
    uint8 x_l = 0U;
    uint8 x_h = 0U;
    uint8 y_l = 0U;
    uint8 y_h = 0U;

    if (0 == motion)
    {
        return 1U;
    }

    if (!driver_pmw3901_status.initialized)
    {
        return 2U;
    }

    motion->motion = driver_pmw3901_register_read(0x02);
    x_l = driver_pmw3901_register_read(0x03);
    x_h = driver_pmw3901_register_read(0x04);
    y_l = driver_pmw3901_register_read(0x05);
    y_h = driver_pmw3901_register_read(0x06);

    motion->delta_x = (int16)(((uint16)x_h << 8) | x_l);
    motion->delta_y = (int16)(((uint16)y_h << 8) | y_l);

    motion->squal = driver_pmw3901_register_read(0x07);
    motion->raw_sum = driver_pmw3901_register_read(0x08);
    motion->max_raw = driver_pmw3901_register_read(0x09);
    motion->min_raw = driver_pmw3901_register_read(0x0A);
    motion->shutter_lower = driver_pmw3901_register_read(0x0B);
    motion->shutter_upper = driver_pmw3901_register_read(0x0C);

    return 0U;
}

uint8 driver_pmw3901_is_ready(void)
{
    return driver_pmw3901_status.initialized;
}

driver_pmw3901_status_struct driver_pmw3901_get_status(void)
{
    return driver_pmw3901_status;
}

void driver_pmw3901_clear_data(driver_pmw3901_data_struct *data)
{
    if (0 == data)
    {
        return;
    }

    memset(data, 0, sizeof(driver_pmw3901_data_struct));
    data->status = driver_pmw3901_get_status();
    data->init_done = data->status.initialized;
    zf_sprintf(data->text, "pmw3901 data cleared");
}

uint8 driver_pmw3901_update_data(driver_pmw3901_data_struct *data)
{
    uint8 ret = 0U;
    driver_pmw3901_motion_struct motion;

    if (0 == data)
    {
        return 1U;
    }

    data->status = driver_pmw3901_get_status();
    data->init_done = data->status.initialized;

    if (!data->init_done)
    {
        data->read_ok = 0U;
        zf_sprintf(data->text, "pmw3901 not ready");
        return 2U;
    }

    ret = driver_pmw3901_read_motion(&motion);
    if (0U != ret)
    {
        data->read_ok = 0U;
        zf_sprintf(data->text, "pmw3901 read error:%d", ret);
        return ret;
    }

    data->motion = motion;
    data->accum_x += motion.delta_x;
    data->accum_y += motion.delta_y;
    data->update_count += 1U;
    data->read_ok = 1U;

    zf_sprintf(data->text,
               "dx:%d dy:%d sumx:%d sumy:%d squal:%d",
               (int32)data->motion.delta_x,
               (int32)data->motion.delta_y,
               data->accum_x,
               data->accum_y,
               (int32)data->motion.squal);
    return 0U;
}

void driver_pmw3901_set_led(uint8 on)
{
    if (!driver_pmw3901_status.initialized)
    {
        return;
    }

    system_delay_ms(200);
    driver_pmw3901_register_write(0x7F, 0x14);
    driver_pmw3901_register_write(0x6F, on ? 0x1CU : 0x00U);
    driver_pmw3901_register_write(0x7F, 0x00);
}
