#ifndef _driver_vl53l0x_h_
#define _driver_vl53l0x_h_

#include "zf_common_typedef.h"
#include "zf_driver_soft_iic.h"

#define DRIVER_VL53L0X_SDA_PIN                     (P11_2)
#define DRIVER_VL53L0X_SCL_PIN                     (P11_1)
#define DRIVER_VL53L0X_I2C_DELAY                   (59U)
#define DRIVER_VL53L0X_I2C_ADDR                    (0x29U)

#ifndef DRIVER_SENSOR_DETECT_UNKNOWN
#define DRIVER_SENSOR_DETECT_UNKNOWN        (0U)
#define DRIVER_SENSOR_DETECT_OK             (1U)
#define DRIVER_SENSOR_DETECT_WIRING_FAULT   (2U)
#define DRIVER_SENSOR_DETECT_CHIP_MISMATCH  (3U)
#define DRIVER_SENSOR_DETECT_INIT_FAILED    (4U)
#endif

#ifndef DRIVER_TOF_SENSOR_UNKNOWN
#define DRIVER_TOF_SENSOR_UNKNOWN           (0U)
#define DRIVER_TOF_SENSOR_VL53L0X           (1U)
#define DRIVER_TOF_SENSOR_VL53L1X           (2U)
#endif

typedef struct
{
    uint16 distance_mm;
    uint8 range_status;
    uint8 data_ready;
} driver_vl53l0x_data_struct;

typedef struct
{
    uint8 initialized;
    uint8 i2c_addr;
    uint8 detect_status;
    uint8 detected_type;
    uint8 expected_model_id;
    uint8 model_id;
    uint8 alt_model_id;
} driver_vl53l0x_status_struct;

uint8 driver_vl53l0x_init(void);
uint8 driver_vl53l0x_read(driver_vl53l0x_data_struct *data);
uint8 driver_vl53l0x_is_ready(void);
driver_vl53l0x_status_struct driver_vl53l0x_get_status(void);
uint8 driver_vl53l0x_start_ranging(void);
uint8 driver_vl53l0x_stop_ranging(void);

#endif
