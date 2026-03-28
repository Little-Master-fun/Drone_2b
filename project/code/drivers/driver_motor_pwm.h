#ifndef _DRIVER_MOTOR_PWM_H_
#define _DRIVER_MOTOR_PWM_H_

#include "zf_common_typedef.h"

#define DRIVER_MOTOR_PWM_COUNT                 (4U)
#define DRIVER_MOTOR_PWM_THROTTLE_MAX          (1200U)

uint8 driver_motor_pwm_init(void);
uint8 driver_motor_pwm_set_throttle(uint8 motor_index, uint16 throttle);
uint8 driver_motor_pwm_set_throttle_all(uint16 m1, uint16 m2, uint16 m3, uint16 m4);
void driver_motor_pwm_stop_all(void);
uint8 driver_motor_pwm_is_ready(void);

#endif
