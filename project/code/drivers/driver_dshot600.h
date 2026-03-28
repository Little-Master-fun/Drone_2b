#ifndef _DRIVER_DSHOT600_H_
#define _DRIVER_DSHOT600_H_

#include "zf_common_typedef.h"

#define DRIVER_DSHOT600_MOTOR_COUNT            (4U)
#define DRIVER_DSHOT600_THROTTLE_MAX           (2047U)

uint8 driver_dshot600_init(void);
uint8 driver_dshot600_set_throttle(uint8 motor_index, uint16 throttle, uint8 telemetry);
uint8 driver_dshot600_set_throttle_all(uint16 m1, uint16 m2, uint16 m3, uint16 m4, uint8 telemetry);
uint8 driver_dshot600_send_frame(void);
uint8 driver_dshot600_send_command(uint16 command, uint8 repeat, uint8 telemetry);
void driver_dshot600_stop_all(void);
uint8 driver_dshot600_is_ready(void);

#endif
