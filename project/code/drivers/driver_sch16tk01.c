#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

#include "driver_sch16tk01.h"

#define DRIVER_SCH16TK01_POWER_ON_TIME_MS         (250U)

#define SCH16T_EOI                               (1U << 1)
#define SCH16T_EN_SENSOR                         (1U << 0)
#define SCH16T_DRY_DRV_EN                        (1U << 5)
#define SCH16T_SPI_SOFT_RESET                    (0x0AU)

#define SCH16T_FILTER_68_HZ                      (0x00U)
#define SCH16T_FILTER_BYPASS                     (0x07U)
#define SCH16T_RATE_RANGE_300                    (0x01U)
#define SCH16T_ACC12_RANGE_80                    (0x01U)
#define SCH16T_ACC3_RANGE_260                    (0x00U)
#define SCH16T_DECIMATION_738_HZ                 (0x04U)

#define SCH16T_RATE_X2                           (0x0AU)
#define SCH16T_RATE_Y2                           (0x0BU)
#define SCH16T_RATE_Z2                           (0x0CU)
#define SCH16T_ACC_X2                            (0x0DU)
#define SCH16T_ACC_Y2                            (0x0EU)
#define SCH16T_ACC_Z2                            (0x0FU)
#define SCH16T_TEMP                              (0x10U)
#define SCH16T_STAT_SUM                          (0x14U)
#define SCH16T_STAT_SUM_SAT                      (0x15U)
#define SCH16T_STAT_COM                          (0x16U)
#define SCH16T_STAT_RATE_COM                     (0x17U)
#define SCH16T_STAT_RATE_X                       (0x18U)
#define SCH16T_STAT_RATE_Y                       (0x19U)
#define SCH16T_STAT_RATE_Z                       (0x1AU)
#define SCH16T_STAT_ACC_X                        (0x1BU)
#define SCH16T_STAT_ACC_Y                        (0x1CU)
#define SCH16T_STAT_ACC_Z                        (0x1DU)
#define SCH16T_CTRL_FILT_RATE                    (0x25U)
#define SCH16T_CTRL_FILT_ACC12                   (0x26U)
#define SCH16T_CTRL_FILT_ACC3                    (0x27U)
#define SCH16T_CTRL_RATE                         (0x28U)
#define SCH16T_CTRL_ACC12                        (0x29U)
#define SCH16T_CTRL_ACC3                         (0x2AU)
#define SCH16T_CTRL_USER_IF                      (0x33U)
#define SCH16T_CTRL_MODE                         (0x35U)
#define SCH16T_CTRL_RESET                        (0x36U)
#define SCH16T_ASIC_ID                           (0x3BU)
#define SCH16T_COMP_ID                           (0x3CU)

#define SPI48_DATA_INT32(a)                     (((int32)(((a) << 4) & 0xFFFFF000ULL)) >> 12)
#define SPI48_DATA_UINT16(a)                    ((uint16)(((a) >> 8) & 0x0000FFFFULL))

typedef struct
{
    uint8 addr;
    uint16 value;
} driver_sch16tk01_register_config_struct;

typedef struct
{
    uint16 summary;
    uint16 saturation;
    uint16 common;
    uint16 rate_common;
    uint16 rate_x;
    uint16 rate_y;
    uint16 rate_z;
    uint16 acc_x;
    uint16 acc_y;
    uint16 acc_z;
} driver_sch16tk01_sensor_status_struct;

static const driver_sch16tk01_register_config_struct driver_sch16tk01_default_config[] = {
    {SCH16T_CTRL_FILT_RATE,  (uint16)((SCH16T_FILTER_68_HZ << 0) | (SCH16T_FILTER_68_HZ << 3) | (SCH16T_FILTER_68_HZ << 6))},
    {SCH16T_CTRL_FILT_ACC12, (uint16)((SCH16T_FILTER_BYPASS << 0) | (SCH16T_FILTER_BYPASS << 3) | (SCH16T_FILTER_BYPASS << 6))},
    {SCH16T_CTRL_FILT_ACC3,  (uint16)((SCH16T_FILTER_BYPASS << 0) | (SCH16T_FILTER_BYPASS << 3) | (SCH16T_FILTER_BYPASS << 6))},
    {SCH16T_CTRL_RATE,       (uint16)((SCH16T_DECIMATION_738_HZ << 0) | (SCH16T_DECIMATION_738_HZ << 3) | (SCH16T_DECIMATION_738_HZ << 6) |
                                      (SCH16T_RATE_RANGE_300 << 9) | (SCH16T_RATE_RANGE_300 << 12))},
    {SCH16T_CTRL_ACC12,      (uint16)((SCH16T_DECIMATION_738_HZ << 0) | (SCH16T_DECIMATION_738_HZ << 3) | (SCH16T_DECIMATION_738_HZ << 6) |
                                      (SCH16T_ACC12_RANGE_80 << 9) | (SCH16T_ACC12_RANGE_80 << 12))},
    {SCH16T_CTRL_ACC3,       (uint16)(SCH16T_ACC3_RANGE_260 << 0)}
};

static uint8 driver_sch16tk01_spi_inited = 0U;
static uint16 driver_sch16tk01_spi_tx_buf[3];
static uint16 driver_sch16tk01_spi_rx_buf[3];

static driver_sch16tk01_sensor_status_struct driver_sch16tk01_sensor_status = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
static driver_sch16tk01_status_struct driver_sch16tk01_status = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

static uint8 driver_sch16tk01_crc8 (uint64 frame)
{
    uint64 data = frame & 0xFFFFFFFFFF00ULL;
    uint8 crc = 0xFFU;
    int32 i = 0;

    for (i = 47; i >= 0; --i)
    {
        uint8 data_bit = (uint8)((data >> i) & 0x01U);
        if (crc & 0x80U)
        {
            crc = (uint8)(((crc << 1) ^ 0x2FU) ^ data_bit);
        }
        else
        {
            crc = (uint8)((crc << 1) | data_bit);
        }
    }

    return crc;
}

static void driver_sch16tk01_spi_init (void)
{
    if (driver_sch16tk01_spi_inited)
    {
        return;
    }

    spi_init(DRIVER_SCH16TK01_SPI_INDEX,
             SPI_MODE0,
             DRIVER_SCH16TK01_SPI_BAUD,
             DRIVER_SCH16TK01_SPI_CLK_PIN,
             DRIVER_SCH16TK01_SPI_MOSI_PIN,
             DRIVER_SCH16TK01_SPI_MISO_PIN,
             SPI_CS_NULL);
    gpio_init(DRIVER_SCH16TK01_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    driver_sch16tk01_spi_inited = 1U;
}

static uint64 driver_sch16tk01_transfer_frame (uint64 frame)
{
    uint64 value = 0U;
    uint32 index = 0U;

    for (index = 0U; index < 3U; ++index)
    {
        driver_sch16tk01_spi_tx_buf[2U - index] = (uint16)((frame >> (index * 16U)) & 0xFFFFU);
    }

    gpio_low(DRIVER_SCH16TK01_CS_PIN);
    system_delay_us(2);
    spi_transfer_16bit(DRIVER_SCH16TK01_SPI_INDEX, driver_sch16tk01_spi_tx_buf, driver_sch16tk01_spi_rx_buf, 3U);
    system_delay_us(2);
    gpio_high(DRIVER_SCH16TK01_CS_PIN);

    for (index = 0U; index < 3U; ++index)
    {
        value |= (uint64)driver_sch16tk01_spi_rx_buf[index] << ((2U - index) * 16U);
    }

    return value;
}

static uint64 driver_sch16tk01_register_read (uint8 addr)
{
    uint64 frame = 0U;

    frame |= (uint64)addr << 38;
    frame |= (uint64)1U << 35;
    frame |= (uint64)driver_sch16tk01_crc8(frame);

    return driver_sch16tk01_transfer_frame(frame);
}

static void driver_sch16tk01_register_write (uint8 addr, uint16 value)
{
    uint64 frame = 0U;

    frame |= (uint64)1U << 37;
    frame |= (uint64)addr << 38;
    frame |= (uint64)1U << 35;
    frame |= (uint64)value << 8;
    frame |= (uint64)driver_sch16tk01_crc8(frame);

    (void)driver_sch16tk01_transfer_frame(frame);
}

static void driver_sch16tk01_software_reset (void)
{
    driver_sch16tk01_register_write(SCH16T_CTRL_RESET, SCH16T_SPI_SOFT_RESET);
}

static uint8 driver_sch16tk01_detect_version (uint16 asic_id, uint16 comp_id)
{
    if (((asic_id == 0x21U) && (comp_id == 0x23U)) ||
        ((asic_id == 0x20U) && (comp_id == 0x17U)))
    {
        return DRIVER_SCH16TK01_CHIP_VERSION_REV1;
    }

    if (((asic_id == 0x21U) && (comp_id == 0x24U)) ||
        ((asic_id == 0x21U) && (comp_id == 0x21U)))
    {
        return DRIVER_SCH16TK01_CHIP_VERSION_REV2;
    }

    return DRIVER_SCH16TK01_CHIP_VERSION_UNKNOWN;
}

static void driver_sch16tk01_probe (void)
{
    uint16 asic_id = 0U;
    uint16 comp_id = 0U;

    (void)driver_sch16tk01_register_read(SCH16T_COMP_ID);
    comp_id = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_ASIC_ID));
    asic_id = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_ASIC_ID));

    driver_sch16tk01_status.asic_id = asic_id;
    driver_sch16tk01_status.comp_id = comp_id;
    driver_sch16tk01_status.chip_version = driver_sch16tk01_detect_version(asic_id, comp_id);

    if (((asic_id == 0x0000U) || (asic_id == 0xFFFFU)) &&
        ((comp_id == 0x0000U) || (comp_id == 0xFFFFU)))
    {
        driver_sch16tk01_status.detect_status = DRIVER_SENSOR_DETECT_WIRING_FAULT;
    }
    else if (driver_sch16tk01_status.chip_version == DRIVER_SCH16TK01_CHIP_VERSION_UNKNOWN)
    {
        driver_sch16tk01_status.detect_status = DRIVER_SENSOR_DETECT_CHIP_MISMATCH;
    }
    else
    {
        driver_sch16tk01_status.detect_status = DRIVER_SENSOR_DETECT_OK;
    }
}

static void driver_sch16tk01_configure (void)
{
    uint32 index = 0U;

    for (index = 0U; index < (sizeof(driver_sch16tk01_default_config) / sizeof(driver_sch16tk01_default_config[0])); ++index)
    {
        driver_sch16tk01_register_write(driver_sch16tk01_default_config[index].addr,
                                        driver_sch16tk01_default_config[index].value);
    }

    driver_sch16tk01_register_write(SCH16T_CTRL_USER_IF, SCH16T_DRY_DRV_EN);
    driver_sch16tk01_register_write(SCH16T_CTRL_MODE, SCH16T_EN_SENSOR);
}

static void driver_sch16tk01_read_status_registers (void)
{
    (void)driver_sch16tk01_register_read(SCH16T_STAT_SUM);
    driver_sch16tk01_sensor_status.summary = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_STAT_SUM_SAT));
    driver_sch16tk01_sensor_status.saturation = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_STAT_COM));
    driver_sch16tk01_sensor_status.common = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_STAT_RATE_COM));
    driver_sch16tk01_sensor_status.rate_common = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_STAT_RATE_X));
    driver_sch16tk01_sensor_status.rate_x = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_STAT_RATE_Y));
    driver_sch16tk01_sensor_status.rate_y = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_STAT_RATE_Z));
    driver_sch16tk01_sensor_status.rate_z = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_STAT_ACC_X));
    driver_sch16tk01_sensor_status.acc_x = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_STAT_ACC_Y));
    driver_sch16tk01_sensor_status.acc_y = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_STAT_ACC_Z));
    driver_sch16tk01_sensor_status.acc_z = SPI48_DATA_UINT16(driver_sch16tk01_register_read(SCH16T_STAT_ACC_Z));

    driver_sch16tk01_status.summary_status = driver_sch16tk01_sensor_status.summary;
    driver_sch16tk01_status.saturation_status = driver_sch16tk01_sensor_status.saturation;
}

static uint8 driver_sch16tk01_validate_sensor_status (void)
{
    uint16 values[10];
    uint32 index = 0U;

    values[0] = driver_sch16tk01_sensor_status.summary;
    values[1] = driver_sch16tk01_sensor_status.saturation;
    values[2] = driver_sch16tk01_sensor_status.common;
    values[3] = driver_sch16tk01_sensor_status.rate_common;
    values[4] = driver_sch16tk01_sensor_status.rate_x;
    values[5] = driver_sch16tk01_sensor_status.rate_y;
    values[6] = driver_sch16tk01_sensor_status.rate_z;
    values[7] = driver_sch16tk01_sensor_status.acc_x;
    values[8] = driver_sch16tk01_sensor_status.acc_y;
    values[9] = driver_sch16tk01_sensor_status.acc_z;

    for (index = 0U; index < 10U; ++index)
    {
        if (values[index] != 0xFFFFU)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8 driver_sch16tk01_validate_register_configuration (void)
{
    uint32 index = 0U;

    for (index = 0U; index < (sizeof(driver_sch16tk01_default_config) / sizeof(driver_sch16tk01_default_config[0])); ++index)
    {
        uint16 value = 0U;

        (void)driver_sch16tk01_register_read(driver_sch16tk01_default_config[index].addr);
        value = SPI48_DATA_UINT16(driver_sch16tk01_register_read(driver_sch16tk01_default_config[index].addr));

        if (value != driver_sch16tk01_default_config[index].value)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8 driver_sch16tk01_validate_frames (const uint64 *values, uint32 count, uint8 *saturation)
{
    static const uint64 mask_general_error =    0x001000000000ULL;
    static const uint64 mask_command_error =    0x000800000000ULL;
    static const uint64 mask_saturation_error = 0x000400000000ULL;
    static const uint64 mask_doing_init =       0x000600000000ULL;
    uint32 index = 0U;

    if (0 != saturation)
    {
        *saturation = 0U;
    }

    for (index = 0U; index < count; ++index)
    {
        uint64 value = values[index];

        if ((value & mask_general_error) || (value & mask_command_error))
        {
            return 0U;
        }

        if ((value & mask_doing_init) == mask_doing_init)
        {
            return 0U;
        }

        if ((value & mask_saturation_error) && (0 != saturation))
        {
            *saturation = 1U;
        }

        if ((uint8)(value & 0xFFU) != driver_sch16tk01_crc8(value))
        {
            return 0U;
        }
    }

    return 1U;
}

uint8 driver_sch16tk01_init (void)
{
    driver_sch16tk01_spi_init();

    driver_sch16tk01_status.initialized = 0U;
    driver_sch16tk01_status.detect_status = DRIVER_SENSOR_DETECT_UNKNOWN;
    driver_sch16tk01_status.chip_version = DRIVER_SCH16TK01_CHIP_VERSION_UNKNOWN;
    driver_sch16tk01_status.last_read_ok = 0U;
    driver_sch16tk01_status.asic_id = 0U;
    driver_sch16tk01_status.comp_id = 0U;
    driver_sch16tk01_status.summary_status = 0U;
    driver_sch16tk01_status.saturation_status = 0U;

    system_delay_ms(DRIVER_SCH16TK01_POWER_ON_TIME_MS);
    driver_sch16tk01_software_reset();
    system_delay_ms(DRIVER_SCH16TK01_POWER_ON_TIME_MS);

    driver_sch16tk01_probe();
    if (driver_sch16tk01_status.detect_status != DRIVER_SENSOR_DETECT_OK)
    {
        return 1U;
    }

    driver_sch16tk01_configure();
    system_delay_ms(DRIVER_SCH16TK01_POWER_ON_TIME_MS);

    driver_sch16tk01_read_status_registers();
    driver_sch16tk01_register_write(SCH16T_CTRL_MODE, (uint16)(SCH16T_EOI | SCH16T_EN_SENSOR));
    system_delay_ms(5);

    driver_sch16tk01_read_status_registers();
    driver_sch16tk01_read_status_registers();

    if ((!driver_sch16tk01_validate_sensor_status()) || (!driver_sch16tk01_validate_register_configuration()))
    {
        driver_sch16tk01_status.detect_status = DRIVER_SENSOR_DETECT_INIT_FAILED;
        return 2U;
    }

    driver_sch16tk01_status.initialized = 1U;
    driver_sch16tk01_status.detect_status = DRIVER_SENSOR_DETECT_OK;
    return 0U;
}

uint8 driver_sch16tk01_read (driver_sch16tk01_data_struct *data)
{
    uint64 gyro_x = 0U;
    uint64 gyro_y = 0U;
    uint64 gyro_z = 0U;
    uint64 acc_x = 0U;
    uint64 acc_y = 0U;
    uint64 acc_z = 0U;
    uint64 temp = 0U;
    uint64 frames[7];
    uint8 saturation = 0U;

    if (0 == data)
    {
        return 1U;
    }

    data->valid = 0U;
    data->saturation = 0U;

    if (!driver_sch16tk01_status.initialized)
    {
        return 2U;
    }

    (void)driver_sch16tk01_register_read(SCH16T_RATE_X2);
    gyro_x = driver_sch16tk01_register_read(SCH16T_RATE_Y2);
    gyro_y = driver_sch16tk01_register_read(SCH16T_RATE_Z2);
    gyro_z = driver_sch16tk01_register_read(SCH16T_ACC_X2);
    acc_x = driver_sch16tk01_register_read(SCH16T_ACC_Y2);
    acc_y = driver_sch16tk01_register_read(SCH16T_ACC_Z2);
    acc_z = driver_sch16tk01_register_read(SCH16T_TEMP);
    temp = driver_sch16tk01_register_read(SCH16T_TEMP);

    frames[0] = gyro_x;
    frames[1] = gyro_y;
    frames[2] = gyro_z;
    frames[3] = acc_x;
    frames[4] = acc_y;
    frames[5] = acc_z;
    frames[6] = temp;

    if (!driver_sch16tk01_validate_frames(frames, 7U, &saturation))
    {
        driver_sch16tk01_status.last_read_ok = 0U;
        return 3U;
    }

    data->gyro_x_raw = SPI48_DATA_INT32(gyro_x);
    data->gyro_y_raw = -SPI48_DATA_INT32(gyro_y);
    data->gyro_z_raw = -SPI48_DATA_INT32(gyro_z);
    data->acc_x_raw = SPI48_DATA_INT32(acc_x);
    data->acc_y_raw = -SPI48_DATA_INT32(acc_y);
    data->acc_z_raw = -SPI48_DATA_INT32(acc_z);
    data->temp_cdeg = SPI48_DATA_INT32(temp) >> 4;
    data->saturation = saturation;
    data->valid = 1U;

    driver_sch16tk01_status.last_read_ok = 1U;
    return 0U;
}

uint8 driver_sch16tk01_is_ready (void)
{
    return driver_sch16tk01_status.initialized;
}

driver_sch16tk01_status_struct driver_sch16tk01_get_status (void)
{
    return driver_sch16tk01_status;
}

uint8 driver_sch16tk01_reset (void)
{
    if (!driver_sch16tk01_spi_inited)
    {
        driver_sch16tk01_spi_init();
    }

    driver_sch16tk01_software_reset();
    system_delay_ms(DRIVER_SCH16TK01_POWER_ON_TIME_MS);
    return 0U;
}
