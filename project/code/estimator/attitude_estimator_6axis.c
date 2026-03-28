#include "math.h"

#include "attitude_estimator_6axis.h"

#ifndef M_PI
#define M_PI 3.1415926f
#endif

#define ATTITUDE_ESTIMATOR_KP                     (2.5f)
#define ATTITUDE_ESTIMATOR_KI                     (0.05f)
#define ATTITUDE_ESTIMATOR_ACCEL_MIN_NORM         (1.0f)
#define ATTITUDE_ESTIMATOR_ACCEL_MAX_NORM         (20.0f)

static attitude_estimator_6axis_state_struct g_attitude_state = {
    {1.0f, 0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.0f, 0.0f, 0U
};
static float g_integral_fb[3] = {0.0f, 0.0f, 0.0f};

static float attitude_estimator_inv_sqrt (float x)
{
    if (x <= 0.0f)
    {
        return 0.0f;
    }

    return 1.0f / sqrtf(x);
}

static void attitude_estimator_quat_to_euler (const float q[4], float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float sin_pitch = 0.0f;

    roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                  1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    sin_pitch = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }
    pitch = asinf(sin_pitch);
    yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));

    *roll_deg = roll * 180.0f / M_PI;
    *pitch_deg = pitch * 180.0f / M_PI;
    *yaw_deg = yaw * 180.0f / M_PI;
    if (*yaw_deg < 0.0f)
    {
        *yaw_deg += 360.0f;
    }
}

void attitude_estimator_6axis_reset (void)
{
    g_attitude_state.q[0] = 1.0f;
    g_attitude_state.q[1] = 0.0f;
    g_attitude_state.q[2] = 0.0f;
    g_attitude_state.q[3] = 0.0f;
    g_attitude_state.roll_deg = 0.0f;
    g_attitude_state.pitch_deg = 0.0f;
    g_attitude_state.yaw_deg = 0.0f;
    g_attitude_state.accel_norm = 0.0f;
    g_attitude_state.valid = 0U;

    g_integral_fb[0] = 0.0f;
    g_integral_fb[1] = 0.0f;
    g_integral_fb[2] = 0.0f;
}

void attitude_estimator_6axis_init (void)
{
    attitude_estimator_6axis_reset();
}

uint8 attitude_estimator_6axis_update (float gx_rad_s,
                                       float gy_rad_s,
                                       float gz_rad_s,
                                       float ax_mss,
                                       float ay_mss,
                                       float az_mss,
                                       float dt_s)
{
    float recip_norm = 0.0f;
    float q0 = g_attitude_state.q[0];
    float q1 = g_attitude_state.q[1];
    float q2 = g_attitude_state.q[2];
    float q3 = g_attitude_state.q[3];
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    float ex = 0.0f;
    float ey = 0.0f;
    float ez = 0.0f;
    float accel_norm_sq = ax_mss * ax_mss + ay_mss * ay_mss + az_mss * az_mss;

    if (dt_s <= 0.0f)
    {
        return 0U;
    }

    g_attitude_state.accel_norm = sqrtf(accel_norm_sq);
    if ((g_attitude_state.accel_norm > ATTITUDE_ESTIMATOR_ACCEL_MIN_NORM) &&
        (g_attitude_state.accel_norm < ATTITUDE_ESTIMATOR_ACCEL_MAX_NORM))
    {
        recip_norm = attitude_estimator_inv_sqrt(accel_norm_sq);
        ax_mss *= recip_norm;
        ay_mss *= recip_norm;
        az_mss *= recip_norm;

        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        ex = (ay_mss * vz - az_mss * vy);
        ey = (az_mss * vx - ax_mss * vz);
        ez = (ax_mss * vy - ay_mss * vx);

        g_integral_fb[0] += ATTITUDE_ESTIMATOR_KI * ex * dt_s;
        g_integral_fb[1] += ATTITUDE_ESTIMATOR_KI * ey * dt_s;
        g_integral_fb[2] += ATTITUDE_ESTIMATOR_KI * ez * dt_s;

        gx_rad_s += ATTITUDE_ESTIMATOR_KP * ex + g_integral_fb[0];
        gy_rad_s += ATTITUDE_ESTIMATOR_KP * ey + g_integral_fb[1];
        gz_rad_s += ATTITUDE_ESTIMATOR_KP * ez + g_integral_fb[2];
    }

    gx_rad_s *= 0.5f * dt_s;
    gy_rad_s *= 0.5f * dt_s;
    gz_rad_s *= 0.5f * dt_s;

    g_attitude_state.q[0] += (-q1 * gx_rad_s - q2 * gy_rad_s - q3 * gz_rad_s);
    g_attitude_state.q[1] += ( q0 * gx_rad_s + q2 * gz_rad_s - q3 * gy_rad_s);
    g_attitude_state.q[2] += ( q0 * gy_rad_s - q1 * gz_rad_s + q3 * gx_rad_s);
    g_attitude_state.q[3] += ( q0 * gz_rad_s + q1 * gy_rad_s - q2 * gx_rad_s);

    recip_norm = attitude_estimator_inv_sqrt(g_attitude_state.q[0] * g_attitude_state.q[0] +
                                             g_attitude_state.q[1] * g_attitude_state.q[1] +
                                             g_attitude_state.q[2] * g_attitude_state.q[2] +
                                             g_attitude_state.q[3] * g_attitude_state.q[3]);
    if (recip_norm <= 0.0f)
    {
        attitude_estimator_6axis_reset();
        return 0U;
    }

    g_attitude_state.q[0] *= recip_norm;
    g_attitude_state.q[1] *= recip_norm;
    g_attitude_state.q[2] *= recip_norm;
    g_attitude_state.q[3] *= recip_norm;
    attitude_estimator_quat_to_euler(g_attitude_state.q,
                                     &g_attitude_state.roll_deg,
                                     &g_attitude_state.pitch_deg,
                                     &g_attitude_state.yaw_deg);
    g_attitude_state.valid = 1U;
    return 1U;
}

attitude_estimator_6axis_state_struct attitude_estimator_6axis_get_state (void)
{
    return g_attitude_state;
}
