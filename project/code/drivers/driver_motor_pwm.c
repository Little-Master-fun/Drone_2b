#include "zf_driver_pwm.h"

#include "driver_motor_pwm.h"

#define DRIVER_MOTOR_PWM_FREQ_HZ              (400U)
#define DRIVER_MOTOR_PWM_MIN_PULSE_US         (1000U)
#define DRIVER_MOTOR_PWM_MAX_PULSE_US         (2000U)
#define DRIVER_MOTOR_PWM_PERIOD_US            (1000000U / DRIVER_MOTOR_PWM_FREQ_HZ)

static const pwm_channel_enum driver_motor_pwm_channels[DRIVER_MOTOR_PWM_COUNT] =
{
    TCPWM_CH09_P05_0,
    TCPWM_CH10_P05_1,
    TCPWM_CH11_P05_2,
    TCPWM_CH12_P05_3
};

static uint8 driver_motor_pwm_initialized = 0U;

static uint32 driver_motor_pwm_throttle_to_duty (uint16 throttle)
{
    uint32 pulse_us = 0U;

    if (throttle == 0U)
    {
        return 0U;
    }

    if (throttle > DRIVER_MOTOR_PWM_THROTTLE_MAX)
    {
        throttle = DRIVER_MOTOR_PWM_THROTTLE_MAX;
    }

    pulse_us = DRIVER_MOTOR_PWM_MIN_PULSE_US +
               ((uint32)throttle * (DRIVER_MOTOR_PWM_MAX_PULSE_US - DRIVER_MOTOR_PWM_MIN_PULSE_US)) /
               DRIVER_MOTOR_PWM_THROTTLE_MAX;

    return (pulse_us * PWM_DUTY_MAX) / DRIVER_MOTOR_PWM_PERIOD_US;
}

uint8 driver_motor_pwm_init (void)
{
    uint8 i = 0U;

    for (i = 0U; i < DRIVER_MOTOR_PWM_COUNT; i++)
    {
        pwm_init(driver_motor_pwm_channels[i], DRIVER_MOTOR_PWM_FREQ_HZ, 0U);
    }

    driver_motor_pwm_initialized = 1U;
    return 0U;
}

uint8 driver_motor_pwm_set_throttle (uint8 motor_index, uint16 throttle)
{
    if ((!driver_motor_pwm_initialized) || (motor_index >= DRIVER_MOTOR_PWM_COUNT))
    {
        return 1U;
    }

    pwm_set_duty(driver_motor_pwm_channels[motor_index], driver_motor_pwm_throttle_to_duty(throttle));
    return 0U;
}

uint8 driver_motor_pwm_set_throttle_all (uint16 m1, uint16 m2, uint16 m3, uint16 m4)
{
    if (!driver_motor_pwm_initialized)
    {
        return 1U;
    }

    pwm_set_duty(driver_motor_pwm_channels[0], driver_motor_pwm_throttle_to_duty(m1));
    pwm_set_duty(driver_motor_pwm_channels[1], driver_motor_pwm_throttle_to_duty(m2));
    pwm_set_duty(driver_motor_pwm_channels[2], driver_motor_pwm_throttle_to_duty(m3));
    pwm_set_duty(driver_motor_pwm_channels[3], driver_motor_pwm_throttle_to_duty(m4));
    return 0U;
}

void driver_motor_pwm_stop_all (void)
{
    if (!driver_motor_pwm_initialized)
    {
        return;
    }

    (void)driver_motor_pwm_set_throttle_all(0U, 0U, 0U, 0U);
}

uint8 driver_motor_pwm_is_ready (void)
{
    return driver_motor_pwm_initialized;
}
