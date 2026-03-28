#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config/flight_params.h"
#include "config/shared_memory.h"
#include "estimator/attitude_estimator_6axis.h"
#include "tasks.h"

#define TASK_ESTIMATOR_NAME            "Estimator"
#define TASK_ESTIMATOR_PERIOD_MS       (5U)
#define TASK_ESTIMATOR_STACK           (768U)
#define TASK_ESTIMATOR_PRIORITY        (6U)

#define DEG2RAD                        (0.01745329252f)
#define GRAVITY_MSS                    (9.80665f)
#define IMU_TIMEOUT_MS                 (60U)
#define FLOW_TIMEOUT_MS                (120U)
#define TOF_TIMEOUT_MS                 (120U)
#define ALT_SOURCE_NONE                (0U)
#define ALT_SOURCE_TOF                 (2U)

typedef struct
{
    uint8 initialized;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float altitude_m;
    float vertical_speed_mps;
    float pos_x_m;
    float pos_y_m;
    float vel_x_mps;
    float vel_y_mps;
    float prev_tof_alt_m;
    uint32 prev_tof_ms;
} estimator_state_struct;

static TaskHandle_t g_task_estimator = 0;
static estimator_state_struct g_est = {0};

static uint32 task_now_ms (void)
{
    return (uint32)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

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

static uint8 task_data_recent (uint32 timestamp_ms, uint32 now_ms, uint32 timeout_ms)
{
    return (((uint32)(now_ms - timestamp_ms)) <= timeout_ms) ? 1U : 0U;
}

static void estimator_reset_state (void)
{
    memset(&g_est, 0, sizeof(g_est));
    attitude_estimator_6axis_reset();
    attitude_estimator_6axis_init();
}

static void estimator_update_attitude (const shm_imu_data_struct *imu, float dt, uint8 *accel_used)
{
    float ax_mss = imu->accel_x_g * GRAVITY_MSS;
    float ay_mss = imu->accel_y_g * GRAVITY_MSS;
    float az_mss = imu->accel_z_g * GRAVITY_MSS;
    attitude_estimator_6axis_state_struct state;

    if (accel_used != 0)
    {
        *accel_used = 0U;
    }

    if (!imu->healthy)
    {
        return;
    }

    (void)attitude_estimator_6axis_update(imu->gyro_x_dps * DEG2RAD,
                                          imu->gyro_y_dps * DEG2RAD,
                                          imu->gyro_z_dps * DEG2RAD,
                                          -ax_mss,
                                          -ay_mss,
                                          -az_mss,
                                          dt);

    state = attitude_estimator_6axis_get_state();
    g_est.roll_rad = state.roll_deg * DEG2RAD;
    g_est.pitch_rad = state.pitch_deg * DEG2RAD;
    g_est.yaw_rad = state.yaw_deg * DEG2RAD;
    if (accel_used != 0)
    {
        *accel_used = state.valid;
    }
}

static void estimator_update_altitude (const shm_tof_data_struct *tof, uint8 tof_enabled, uint32 now_ms, uint8 *alt_rejected, uint8 *alt_source)
{
    float z_meas = 0.0f;
    float dt = 0.0f;
    float vz_meas = 0.0f;
    float alpha = 0.0f;

    if (alt_rejected != 0)
    {
        *alt_rejected = 0U;
    }
    if (alt_source != 0)
    {
        *alt_source = ALT_SOURCE_NONE;
    }

    if ((!tof_enabled) || (!tof->healthy) || (!tof->data_ready) || (!task_data_recent(tof->timestamp_ms, now_ms, TOF_TIMEOUT_MS)))
    {
        return;
    }

    z_meas = (float)tof->distance_mm * 0.001f;
    if ((z_meas < 0.05f) || (z_meas > 4.5f))
    {
        if (alt_rejected != 0)
        {
            *alt_rejected = 1U;
        }
        return;
    }

    if (!g_est.initialized)
    {
        g_est.altitude_m = z_meas;
        g_est.prev_tof_alt_m = z_meas;
        g_est.prev_tof_ms = tof->timestamp_ms;
        g_est.initialized = 1U;
    }

    if ((g_est.prev_tof_ms != 0U) && (tof->timestamp_ms > g_est.prev_tof_ms))
    {
        dt = ((float)(tof->timestamp_ms - g_est.prev_tof_ms)) * 0.001f;
        if (dt > 0.002f)
        {
            vz_meas = (z_meas - g_est.prev_tof_alt_m) / dt;
            vz_meas = task_clamp_float(vz_meas, -3.0f, 3.0f);
            g_est.vertical_speed_mps += (vz_meas - g_est.vertical_speed_mps) * 0.25f;
        }
    }

    alpha = 0.20f;
    g_est.altitude_m += (z_meas - g_est.altitude_m) * alpha;
    g_est.prev_tof_alt_m = z_meas;
    g_est.prev_tof_ms = tof->timestamp_ms;

    if (alt_source != 0)
    {
        *alt_source = ALT_SOURCE_TOF;
    }
}

static uint8 estimator_update_flow_xy (const shm_flow_data_struct *flow,
                                       const shm_tof_data_struct *tof,
                                       float flow_scale_rad_per_count,
                                       float dt,
                                       uint32 now_ms,
                                       uint8 *flow_rejected)
{
    float z_m = 0.0f;
    float flow_x_rad_s = 0.0f;
    float flow_y_rad_s = 0.0f;
    float vx_body = 0.0f;
    float vy_body = 0.0f;
    float cos_yaw = 0.0f;
    float sin_yaw = 0.0f;
    float vx_meas = 0.0f;
    float vy_meas = 0.0f;
    float innov_x = 0.0f;
    float innov_y = 0.0f;
    float innov_norm = 0.0f;

    if (flow_rejected != 0)
    {
        *flow_rejected = 0U;
    }

    if ((!flow->healthy) || (!tof->healthy) || (!tof->data_ready))
    {
        return 0U;
    }

    if ((!task_data_recent(flow->timestamp_ms, now_ms, FLOW_TIMEOUT_MS)) ||
        (!task_data_recent(tof->timestamp_ms, now_ms, TOF_TIMEOUT_MS)))
    {
        return 0U;
    }

    if (flow->squal < 12U)
    {
        return 0U;
    }

    z_m = (float)tof->distance_mm * 0.001f;
    if ((z_m < 0.06f) || (z_m > 4.5f))
    {
        return 0U;
    }

    if (dt < 0.002f)
    {
        dt = 0.002f;
    }

    flow_x_rad_s = ((float)flow->delta_x * flow_scale_rad_per_count) / dt;
    flow_y_rad_s = ((float)flow->delta_y * flow_scale_rad_per_count) / dt;

    vx_body = -flow_y_rad_s * z_m;
    vy_body = flow_x_rad_s * z_m;

    cos_yaw = cosf(g_est.yaw_rad);
    sin_yaw = sinf(g_est.yaw_rad);
    vx_meas = cos_yaw * vx_body - sin_yaw * vy_body;
    vy_meas = sin_yaw * vx_body + cos_yaw * vy_body;

    innov_x = vx_meas - g_est.vel_x_mps;
    innov_y = vy_meas - g_est.vel_y_mps;
    innov_norm = sqrtf(innov_x * innov_x + innov_y * innov_y);
    if (innov_norm > 2.5f)
    {
        if (flow_rejected != 0)
        {
            *flow_rejected = 1U;
        }
        return 0U;
    }

    g_est.vel_x_mps += innov_x * 0.35f;
    g_est.vel_y_mps += innov_y * 0.35f;
    g_est.pos_x_m += g_est.vel_x_mps * dt;
    g_est.pos_y_m += g_est.vel_y_mps * dt;
    return 1U;
}

static void estimator_task_entry (void *parameter)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_tick = last_wake;

    (void)parameter;

    estimator_reset_state();

    while (1)
    {
        shm_imu_data_struct imu = {0};
        shm_flow_data_struct flow = {0};
        shm_tof_data_struct tof = {0};
        shm_estimator_data_struct out = {0};
        TickType_t now_tick = xTaskGetTickCount();
        uint32 now_ms = task_now_ms();
        float dt = (float)(now_tick - last_tick) * 0.001f;
        uint8 accel_used = 0U;
        uint8 flow_used = 0U;
        uint8 flow_rejected = 0U;
        uint8 alt_rejected = 0U;
        uint8 altitude_source = ALT_SOURCE_NONE;
        uint8 flow_enabled = flight_param_get_u8_default(FLIGHT_PARAM_FLOW_ENABLE, 1U);
        uint8 tof_enabled = flight_param_get_u8_default(FLIGHT_PARAM_TOF_ENABLE, 1U);
        float flow_scale = flight_param_get_f32_default(FLIGHT_PARAM_FLOW_RAD_PER_COUNT, 0.0012f);

        if (dt <= 0.0f)
        {
            dt = (float)TASK_ESTIMATOR_PERIOD_MS * 0.001f;
        }
        if (dt > 0.02f)
        {
            dt = 0.02f;
        }
        last_tick = now_tick;

        (void)shm_read_imu(0U, &imu, 0);
        (void)shm_read_flow(&flow, 0);
        (void)shm_read_tof(&tof, 0);

        if (imu.healthy && task_data_recent(imu.timestamp_ms, now_ms, IMU_TIMEOUT_MS))
        {
            estimator_update_attitude(&imu, dt, &accel_used);
        }

        estimator_update_altitude(&tof, tof_enabled, now_ms, &alt_rejected, &altitude_source);

        if (flow_enabled)
        {
            flow_used = estimator_update_flow_xy(&flow, &tof, flow_scale, dt, now_ms, &flow_rejected);
        }
        else
        {
            g_est.vel_x_mps *= 0.98f;
            g_est.vel_y_mps *= 0.98f;
        }

        out.timestamp_ms = now_ms;
        out.healthy = (imu.healthy && task_data_recent(imu.timestamp_ms, now_ms, IMU_TIMEOUT_MS)) ? 1U : 0U;
        out.flow_fused = flow_used;
        out.active_imu = 0U;
        out.altitude_source = altitude_source;
        out.accel_fused = accel_used;
        out.alt_rejected = alt_rejected;
        out.flow_rejected = flow_rejected;
        out.roll_rad = g_est.roll_rad;
        out.pitch_rad = g_est.pitch_rad;
        out.yaw_rad = g_est.yaw_rad;
        out.altitude_m = g_est.altitude_m;
        out.vertical_speed_mps = g_est.vertical_speed_mps;
        out.pos_x_m = g_est.pos_x_m;
        out.pos_y_m = g_est.pos_y_m;
        out.vel_x_mps = g_est.vel_x_mps;
        out.vel_y_mps = g_est.vel_y_mps;

        shm_publish_estimator(&out);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_ESTIMATOR_PERIOD_MS));
    }
}

void tasks_estimator_init (void)
{
    if (g_task_estimator == 0)
    {
        xTaskCreate(estimator_task_entry, TASK_ESTIMATOR_NAME, TASK_ESTIMATOR_STACK, 0, TASK_ESTIMATOR_PRIORITY, &g_task_estimator);
    }
}
