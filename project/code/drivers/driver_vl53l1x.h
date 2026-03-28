#ifndef _driver_vl53l1x_h_
#define _driver_vl53l1x_h_

#include "zf_common_typedef.h"
#include "zf_device_dl1b.h"

#define DRIVER_VL53L1X_SDA_PIN                     (DL1B_SDA_PIN)
#define DRIVER_VL53L1X_SCL_PIN                     (DL1B_SCL_PIN)
#define DRIVER_VL53L1X_I2C_DELAY                   (DL1B_SOFT_IIC_DELAY)
#define DRIVER_VL53L1X_I2C_ADDR                    (DL1B_DEV_ADDR)
#define DRIVER_VL53L1X_XS_PIN                      (DL1B_XS_PIN)
#define DRIVER_VL53L1X_EXPECTED_MODEL_ID           (0xEAU)

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
} driver_vl53l1x_data_struct;

typedef struct
{
    uint8 initialized;
    uint8 i2c_addr;
    uint8 detect_status;
    uint8 detected_type;
    uint8 expected_model_id;
    uint8 model_id;
    uint8 boot_state;
} driver_vl53l1x_status_struct;

uint8 driver_vl53l1x_init(void);
uint8 driver_vl53l1x_read(driver_vl53l1x_data_struct *data);
uint8 driver_vl53l1x_is_ready(void);
driver_vl53l1x_status_struct driver_vl53l1x_get_status(void);
uint8 driver_vl53l1x_start_ranging(void);
uint8 driver_vl53l1x_stop_ranging(void);

#endif
