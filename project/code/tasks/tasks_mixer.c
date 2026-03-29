#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config/flight_params.h"
#include "config/shared_memory.h"
#include "drivers/driver_motor_pwm.h"
#include "tasks.h"

#define TASK_MIXER_NAME                "Mixer"
#define TASK_MIXER_PERIOD_MS           (2U)
#define TASK_MIXER_STACK               (384U)
#define TASK_MIXER_PRIORITY            (7U)

#define MIXER_OUTPUT_MIN               (0.0f)
#define MIXER_OUTPUT_MAX               (1200.0f)
#define MIXER_YAW_MARGIN_RATIO         (0.15f)

static TaskHandle_t g_task_mixer = 0;
static uint8 g_motor_output_ready = 0U;

static float task_clamp_float (float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint16 task_throttle_to_pwm (float throttle)
{
    return (uint16)task_clamp_float(throttle, MIXER_OUTPUT_MIN, MIXER_OUTPUT_MAX);
}

static uint32 task_now_ms (void)
{
    return (uint32)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void mixer_load_output_map (uint8 physical_to_logical[4])
{
    uint8 used_mask = 0U;
    uint8 i = 0U;

    physical_to_logical[0] = flight_param_get_u8_default(FLIGHT_PARAM_MOTOR_MAP_OUT1, 1U);
    physical_to_logical[1] = flight_param_get_u8_default(FLIGHT_PARAM_MOTOR_MAP_OUT2, 2U);
    physical_to_logical[2] = flight_param_get_u8_default(FLIGHT_PARAM_MOTOR_MAP_OUT3, 3U);
    physical_to_logical[3] = flight_param_get_u8_default(FLIGHT_PARAM_MOTOR_MAP_OUT4, 4U);

    for (i = 0U; i < 4U; i++)
    {
        uint8 logical_index = physical_to_logical[i];

        if ((logical_index < 1U) || (logical_index > 4U))
        {
            break;
        }

        logical_index = (uint8)(logical_index - 1U);
        if ((used_mask & (1U << logical_index)) != 0U)
        {
            break;
        }

        used_mask |= (1U << logical_index);
        physical_to_logical[i] = logical_index;
    }

    if (used_mask != 0x0FU)
    {
        physical_to_logical[0] = 0U;
        physical_to_logical[1] = 1U;
        physical_to_logical[2] = 2U;
        physical_to_logical[3] = 3U;
    }
}

static void mixer_map_logical_to_physical (const float logical[4], const uint8 physical_to_logical[4], float physical[4])
{
    uint8 i = 0U;

    for (i = 0U; i < 4U; i++)
    {
        physical[i] = logical[physical_to_logical[i]];
    }
}

static float mixer_compute_desaturation_gain (const float motors[4],
                                              const float desaturation_vector[4],
                                              float min_output,
                                              float max_output)
{
    float k_min = 0.0f;
    float k_max = 0.0f;
    uint8 i = 0U;

    for (i = 0U; i < 4U; i++)
    {
        float vec = desaturation_vector[i];

        if (fabsf(vec) < 1e-5f)
        {
            continue;
        }

        if (motors[i] < min_output)
        {
            float k = (min_output - motors[i]) / vec;
            if (k < k_min) { k_min = k; }
            if (k > k_max) { k_max = k; }
        }

        if (motors[i] > max_output)
        {
            float k = (max_output - motors[i]) / vec;
            if (k < k_min) { k_min = k; }
            if (k > k_max) { k_max = k; }
        }
    }

    return k_min + k_max;
}

static float mixer_apply_desaturation (float motors[4],
                                       const float desaturation_vector[4],
                                       float min_output,
                                       float max_output,
                                       uint8 allow_positive_gain,
                                       uint8 allow_negative_gain)
{
    float total_gain = 0.0f;
    float gain = mixer_compute_desaturation_gain(motors, desaturation_vector, min_output, max_output);
    uint8 i = 0U;

    if ((gain > 0.0f) && (!allow_positive_gain))
    {
        return 0.0f;
    }
    if ((gain < 0.0f) && (!allow_negative_gain))
    {
        return 0.0f;
    }

    for (i = 0U; i < 4U; i++)
    {
        motors[i] += gain * desaturation_vector[i];
    }
    total_gain += gain;

    gain = 0.5f * mixer_compute_desaturation_gain(motors, desaturation_vector, min_output, max_output);
    if (((gain > 0.0f) && allow_positive_gain) || ((gain < 0.0f) && allow_negative_gain))
    {
        for (i = 0U; i < 4U; i++)
        {
            motors[i] += gain * desaturation_vector[i];
        }
        total_gain += gain;
    }

    return total_gain;
}

static void mixer_task_entry (void *parameter)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)parameter;

    while (1)
    {
        shm_control_data_struct control = {0};
        shm_mixer_data_struct out = {0};
        float test_thr = 0.0f;
        float yaw_sign_m1 = 1.0f;
        float yaw_sign_m2 = -1.0f;
        float yaw_sign_m3 = 1.0f;
        float yaw_sign_m4 = -1.0f;
        float thrust_vec[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        float roll_vec[4] = {1.0f, -1.0f, -1.0f, 1.0f};
        float pitch_vec[4] = {-1.0f, -1.0f, 1.0f, 1.0f};
        float yaw_vec[4] = {0.0f};
        float motors[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float physical_motors[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float collective_gain = 0.0f;
        float yaw_gain = 0.0f;
        float yaw_cmd = 0.0f;
        float yaw_max = MIXER_OUTPUT_MAX + (MIXER_OUTPUT_MAX - MIXER_OUTPUT_MIN) * MIXER_YAW_MARGIN_RATIO;
        uint8 physical_to_logical[4] = {0U, 1U, 2U, 3U};
        uint8 motor_dir_m1 = flight_param_get_u8_default(FLIGHT_PARAM_MOTOR_DIR_M1, 0U);
        uint8 motor_dir_m2 = flight_param_get_u8_default(FLIGHT_PARAM_MOTOR_DIR_M2, 0U);
        uint8 motor_dir_m3 = flight_param_get_u8_default(FLIGHT_PARAM_MOTOR_DIR_M3, 0U);
        uint8 motor_dir_m4 = flight_param_get_u8_default(FLIGHT_PARAM_MOTOR_DIR_M4, 0U);
        uint8 motor_test_enable = flight_param_get_u8_default(FLIGHT_PARAM_MOTOR_TEST_ENABLE, 0U);
        uint8 motor_test_index = flight_param_get_u8_default(FLIGHT_PARAM_MOTOR_TEST_INDEX, 0U);
        uint8 i = 0U;

        (void)shm_read_control(&control, 0);
        test_thr = flight_param_get_f32_default(FLIGHT_PARAM_MOTOR_TEST_THR, 150.0f);

        if (motor_dir_m1) { yaw_sign_m1 = -yaw_sign_m1; }
        if (motor_dir_m2) { yaw_sign_m2 = -yaw_sign_m2; }
        if (motor_dir_m3) { yaw_sign_m3 = -yaw_sign_m3; }
        if (motor_dir_m4) { yaw_sign_m4 = -yaw_sign_m4; }
        yaw_vec[0] = yaw_sign_m1;
        yaw_vec[1] = yaw_sign_m2;
        yaw_vec[2] = yaw_sign_m3;
        yaw_vec[3] = yaw_sign_m4;
        mixer_load_output_map(physical_to_logical);

        out.timestamp_ms = task_now_ms();
        out.enabled = g_motor_output_ready ? 1U : 0U;
        out.yaw_scale = 1.0f;

        if (out.enabled && motor_test_enable && (!control.armed))
        {
            if (motor_test_index == 1U) { motors[0] = test_thr; }
            else if (motor_test_index == 2U) { motors[1] = test_thr; }
            else if (motor_test_index == 3U) { motors[2] = test_thr; }
            else if (motor_test_index == 4U) { motors[3] = test_thr; }
            else if (motor_test_index == 5U) { motors[0] = test_thr; motors[1] = test_thr; motors[2] = test_thr; motors[3] = test_thr; }

            mixer_map_logical_to_physical(motors, physical_to_logical, physical_motors);
            out.motor1 = task_throttle_to_pwm(physical_motors[0]);
            out.motor2 = task_throttle_to_pwm(physical_motors[1]);
            out.motor3 = task_throttle_to_pwm(physical_motors[2]);
            out.motor4 = task_throttle_to_pwm(physical_motors[3]);
            (void)driver_motor_pwm_set_throttle_all(out.motor1, out.motor2, out.motor3, out.motor4);
        }
        else if (out.enabled && control.armed && !control.failsafe)
        {
            motors[0] = control.throttle + control.roll_cmd * roll_vec[0] + control.pitch_cmd * pitch_vec[0];
            motors[1] = control.throttle + control.roll_cmd * roll_vec[1] + control.pitch_cmd * pitch_vec[1];
            motors[2] = control.throttle + control.roll_cmd * roll_vec[2] + control.pitch_cmd * pitch_vec[2];
            motors[3] = control.throttle + control.roll_cmd * roll_vec[3] + control.pitch_cmd * pitch_vec[3];

            collective_gain += mixer_apply_desaturation(motors, thrust_vec, MIXER_OUTPUT_MIN, MIXER_OUTPUT_MAX, 0U, 1U);
            (void)mixer_apply_desaturation(motors, roll_vec, MIXER_OUTPUT_MIN, MIXER_OUTPUT_MAX, 1U, 1U);
            (void)mixer_apply_desaturation(motors, pitch_vec, MIXER_OUTPUT_MIN, MIXER_OUTPUT_MAX, 1U, 1U);

            yaw_cmd = control.yaw_cmd;
            for (i = 0U; i < 4U; i++)
            {
                motors[i] += yaw_cmd * yaw_vec[i];
            }

            yaw_gain = mixer_apply_desaturation(motors, yaw_vec, MIXER_OUTPUT_MIN, yaw_max, 1U, 1U);
            collective_gain += mixer_apply_desaturation(motors, thrust_vec, MIXER_OUTPUT_MIN, MIXER_OUTPUT_MAX, 0U, 1U);

            if (fabsf(yaw_cmd) > 1e-4f)
            {
                out.yaw_scale = task_clamp_float((yaw_cmd + yaw_gain) / yaw_cmd, 0.0f, 1.2f);
            }

            out.yaw_limited = (fabsf(yaw_gain) > 1e-4f) ? 1U : 0U;
            out.thrust_reduced = (collective_gain < -1e-4f) ? 1U : 0U;
            out.collective_offset = collective_gain;

            for (i = 0U; i < 4U; i++)
            {
                if (motors[i] > MIXER_OUTPUT_MAX)
                {
                    out.saturation_high = 1U;
                }
                if (motors[i] < MIXER_OUTPUT_MIN)
                {
                    out.saturation_low = 1U;
                }
                motors[i] = task_clamp_float(motors[i], MIXER_OUTPUT_MIN, MIXER_OUTPUT_MAX);
            }

            mixer_map_logical_to_physical(motors, physical_to_logical, physical_motors);
            out.motor1 = task_throttle_to_pwm(physical_motors[0]);
            out.motor2 = task_throttle_to_pwm(physical_motors[1]);
            out.motor3 = task_throttle_to_pwm(physical_motors[2]);
            out.motor4 = task_throttle_to_pwm(physical_motors[3]);
            (void)driver_motor_pwm_set_throttle_all(out.motor1, out.motor2, out.motor3, out.motor4);
        }
        else
        {
            out.enabled = 0U;
            driver_motor_pwm_stop_all();
        }

        shm_publish_mixer(&out);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_MIXER_PERIOD_MS));
    }
}

void tasks_mixer_init (void)
{
    if (g_task_mixer == 0)
    {
        if (!g_motor_output_ready)
        {
            g_motor_output_ready = (driver_motor_pwm_init() == 0U) ? 1U : 0U;
        }
        xTaskCreate(mixer_task_entry, TASK_MIXER_NAME, TASK_MIXER_STACK, 0, TASK_MIXER_PRIORITY, &g_task_mixer);
    }
}
