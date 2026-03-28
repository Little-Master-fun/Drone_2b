#ifndef _driver_sch16tk01_h_
#define _driver_sch16tk01_h_

#include "zf_common_typedef.h"
#include "zf_driver_spi.h"

#define DRIVER_SCH16TK01_SPI_INDEX                (SPI_1)
#define DRIVER_SCH16TK01_SPI_BAUD                 (5000000U)
#define DRIVER_SCH16TK01_SPI_CLK_PIN              (SPI1_CLK_P06_2)
#define DRIVER_SCH16TK01_SPI_MOSI_PIN             (SPI1_MOSI_P06_1)
#define DRIVER_SCH16TK01_SPI_MISO_PIN             (SPI1_MISO_P06_0)
#define DRIVER_SCH16TK01_CS_PIN                   (P06_3)

#define DRIVER_SCH16TK01_CHIP_VERSION_UNKNOWN     (0U)
#define DRIVER_SCH16TK01_CHIP_VERSION_REV1        (1U)
#define DRIVER_SCH16TK01_CHIP_VERSION_REV2        (2U)

#ifndef DRIVER_SENSOR_DETECT_UNKNOWN
#define DRIVER_SENSOR_DETECT_UNKNOWN        (0U)
#define DRIVER_SENSOR_DETECT_OK             (1U)
#define DRIVER_SENSOR_DETECT_WIRING_FAULT   (2U)
#define DRIVER_SENSOR_DETECT_CHIP_MISMATCH  (3U)
#define DRIVER_SENSOR_DETECT_INIT_FAILED    (4U)
#endif

typedef struct
{
    int32 acc_x_raw;
    int32 acc_y_raw;
    int32 acc_z_raw;
    int32 gyro_x_raw;
    int32 gyro_y_raw;
    int32 gyro_z_raw;
    int32 temp_cdeg;
    uint8 saturation;
    uint8 valid;
} driver_sch16tk01_data_struct;

typedef struct
{
    uint8 initialized;
    uint8 detect_status;
    uint8 chip_version;
    uint8 last_read_ok;
    uint16 asic_id;
    uint16 comp_id;
    uint16 summary_status;
    uint16 saturation_status;
} driver_sch16tk01_status_struct;

uint8 driver_sch16tk01_init(void);
uint8 driver_sch16tk01_read(driver_sch16tk01_data_struct *data);
uint8 driver_sch16tk01_is_ready(void);
driver_sch16tk01_status_struct driver_sch16tk01_get_status(void);
uint8 driver_sch16tk01_reset(void);

#endif
