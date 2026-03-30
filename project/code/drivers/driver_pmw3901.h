#ifndef _driver_pmw3901_h_
#define _driver_pmw3901_h_

#include "zf_common_typedef.h"
#include "zf_driver_exti.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

#define DRIVER_PMW3901_SPI_INDEX          (SPI_3)
#define DRIVER_PMW3901_SPI_BAUD           (2U * 1000U * 1000U)
#define DRIVER_PMW3901_SPI_CLK_PIN        (SPI3_CLK_P13_2)
#define DRIVER_PMW3901_SPI_MOSI_PIN       (SPI3_MOSI_P13_1)
#define DRIVER_PMW3901_SPI_MISO_PIN       (SPI3_MISO_P13_0)
#define DRIVER_PMW3901_CS_PIN             (P13_3)
#define DRIVER_PMW3901_USE_MOTION_IRQ     (1U)
#define DRIVER_PMW3901_MOT_PIN            (P14_3)
#define DRIVER_PMW3901_MOT_TRIGGER        (EXTI_TRIGGER_RISING)

#ifndef DRIVER_SENSOR_DETECT_UNKNOWN
#define DRIVER_SENSOR_DETECT_UNKNOWN        (0U)
#define DRIVER_SENSOR_DETECT_OK             (1U)
#define DRIVER_SENSOR_DETECT_WIRING_FAULT   (2U)
#define DRIVER_SENSOR_DETECT_CHIP_MISMATCH  (3U)
#define DRIVER_SENSOR_DETECT_INIT_FAILED    (4U)
#endif

typedef struct
{
    int16 delta_x;
    int16 delta_y;
    uint8 motion;
    uint8 squal;
    uint8 raw_sum;
    uint8 max_raw;
    uint8 min_raw;
    uint8 shutter_upper;
    uint8 shutter_lower;
} driver_pmw3901_motion_struct;

typedef struct
{
    uint8 initialized;
    uint8 detect_status;
    uint8 expected_chip_id;
    uint8 chip_id;
    uint8 chip_id_inverse;
} driver_pmw3901_status_struct;

typedef struct
{
    driver_pmw3901_status_struct status;
    driver_pmw3901_motion_struct motion;
    int32 accum_x;
    int32 accum_y;
    uint32 update_count;
    uint8 init_done;
    uint8 read_ok;
    int8 text[96];
} driver_pmw3901_data_struct;

uint8 driver_pmw3901_init(void);
uint8 driver_pmw3901_read_motion(driver_pmw3901_motion_struct *motion);
uint8 driver_pmw3901_is_ready(void);
driver_pmw3901_status_struct driver_pmw3901_get_status(void);
void driver_pmw3901_clear_data(driver_pmw3901_data_struct *data);
uint8 driver_pmw3901_update_data(driver_pmw3901_data_struct *data);
void driver_pmw3901_set_led(uint8 on);
void driver_pmw3901_motion_irq_handler(void);
uint8 driver_pmw3901_motion_irq_take(uint8 clear_flag);

#endif
