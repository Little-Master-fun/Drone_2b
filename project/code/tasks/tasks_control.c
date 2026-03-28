#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config/flight_params.h"
#include "config/shared_memory.h"
#include "tasks.h"

#define TASK_CONTROL_NAME              "Control"
#define TASK_CONTROL_PERIOD_MS         (4U)
#define TASK_CONTROL_STACK             (768U)
#define TASK_CONTROL_PRIORITY          (6U)

#define DEG2RAD                        (0.01745329252f)
#define IMU_TIMEOUT_MS                 (60U)
#define GRAVITY_MSS                    (9.80665f)

#define FLIGHT_MODE_STABILIZE_TEST     (0U)
#define FLIGHT_MODE_POSHOLD_FLOW_TOF   (1U)
#define SHM_NAV_STATE_NONE             (0U)
#define SHM_NAV_STATE_HOLD             (1U)
#define SHM_NAV_STATE_LAND             (2U)
#define SHM_LAND_STATE_GROUND_CONTACT  (1U)
#define SHM_LAND_STATE_MAYBE_LANDED    (2U)

#define CONTROL_TELEM_FLAG_MANUAL_THR  (1U << 0)
#define CONTROL_TELEM_FLAG_ALT_AW_HIGH (1U << 1)
#define CONTROL_TELEM_FLAG_ALT_AW_LOW  (1U << 2)
#define CONTROL_TELEM_FLAG_RATE_AW_RP  (1U << 3)
#define CONTROL_TELEM_FLAG_RATE_AW_YAW (1U << 4)
#define CONTROL_TELEM_FLAG_HTE_VALID   (1U << 5)

#define HTE_STATUS_ENABLED             (1U << 0)
#define HTE_STATUS_CONDITIONS_OK       (1U << 1)
#define HTE_STATUS_FUSED               (1U << 2)
#define HTE_STATUS_VALID               (1U << 3)
#define HTE_STATUS_INNOV_REJECTED      (1U << 4)
#define HTE_STATUS_STALE               (1U << 5)

#define HTE_VALID_WINDOW_MS            (1200U)
#define HTE_VALID_STALE_MS             (350U)
#define HTE_VALID_MIN_SAMPLES          (12U)
#define HTE_VALID_MIN_ACCEPT_RATIO     (0.65f)

static TaskHandle_t g_task_control = 0;

static float g_rate_i_roll = 0.0f;
static float g_rate_i_pitch = 0.0f;
static float g_rate_i_yaw = 0.0f;
static float g_prev_rate_err_roll = 0.0f;
static float g_prev_rate_err_pitch = 0.0f;
static float g_prev_rate_err_yaw = 0.0f;
static float g_alt_i_state = 0.0f;
static float g_prev_vel_z_error = 0.0f;

static uint8 g_hold_sp_valid = 0U;
static float g_hold_x_sp_m = 0.0f;
static float g_hold_y_sp_m = 0.0f;
static uint8 g_host_alt_sp_valid = 0U;
static float g_host_alt_sp_m = 0.0f;
static uint8 g_host_yaw_sp_valid = 0U;
static float g_host_yaw_sp_rad = 0.0f;

static float g_hover_thrust_est = 350.0f;
static float g_hover_thrust_var = 6400.0f;
static float g_prev_vz_est = 0.0f;
static float g_acc_z_lpf = 0.0f;
static uint8 g_hover_est_valid = 0U;
static uint8 g_hover_est_initialized = 0U;
static uint32 g_hover_last_fuse_ms = 0U;
static uint32 g_hover_window_start_ms = 0U;
static uint16 g_hover_window_attempts = 0U;
static uint16 g_hover_window_accepts = 0U;
static uint16 g_hover_window_rejects = 0U;
static float g_hover_innov = 0.0f;
static float g_hover_innov_var = 0.0f;
static float g_hover_test_ratio = 0.0f;
static float g_hover_meas_acc = 0.0f;
static float g_hover_meas_noise = 0.0f;
static uint8 g_hover_status = 0U;

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

static uint32 task_now_ms (void)
{
    return (uint32)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static float task_wrap_pi (float angle_rad)
{
    while (angle_rad > 3.14159265359f)
    {
        angle_rad -= 6.28318530718f;
    }
    while (angle_rad < -3.14159265359f)
    {
        angle_rad += 6.28318530718f;
    }
    return angle_rad;
}

static void control_reset_rate_pid_state (void)
{
    g_rate_i_roll = 0.0f;
    g_rate_i_pitch = 0.0f;
    g_rate_i_yaw = 0.0f;
    g_prev_rate_err_roll = 0.0f;
    g_prev_rate_err_pitch = 0.0f;
    g_prev_rate_err_yaw = 0.0f;
}

static void control_reset_altitude_state (void)
{
    g_alt_i_state = 0.0f;
    g_prev_vel_z_error = 0.0f;
}

static uint8 control_imu_recent (const shm_imu_data_struct *imu, uint32 now_ms)
{
    if ((imu == 0) || (!imu->healthy))
    {
        return 0U;
    }

    if (((uint32)(now_ms - imu->timestamp_ms)) > IMU_TIMEOUT_MS)
    {
        return 0U;
    }

    return 1U;
}

static uint8 control_host_recent (const shm_host_control_data_struct *host, uint32 now_ms, uint32 timeout_ms)
{
    if ((host == 0) || (!host->link_active))
    {
        return 0U;
    }

    return (((uint32)(now_ms - host->timestamp_ms)) <= timeout_ms) ? 1U : 0U;
}

static uint8 control_get_body_rates_rps (float *wx, float *wy, float *wz, uint32 now_ms)
{
    shm_imu_data_struct imu = {0};

    if ((wx == 0) || (wy == 0) || (wz == 0))
    {
        return 0U;
    }

    (void)shm_read_imu(0U, &imu, 0);
    if (!control_imu_recent(&imu, now_ms))
    {
        return 0U;
    }

    *wx = imu.gyro_x_dps * DEG2RAD;
    *wy = imu.gyro_y_dps * DEG2RAD;
    *wz = imu.gyro_z_dps * DEG2RAD;
    return 1U;
}

static void control_reset_hover_thrust_estimator (float hover_throttle, float init_std)
{
    if (init_std < 1.0f)
    {
        init_std = 1.0f;
    }

    g_hover_thrust_est = hover_throttle;
    g_hover_thrust_var = init_std * init_std;
    g_prev_vz_est = 0.0f;
    g_acc_z_lpf = 0.0f;
    g_hover_est_valid = 0U;
    g_hover_est_initialized = 1U;
    g_hover_last_fuse_ms = 0U;
    g_hover_window_start_ms = 0U;
    g_hover_window_attempts = 0U;
    g_hover_window_accepts = 0U;
    g_hover_window_rejects = 0U;
    g_hover_innov = 0.0f;
    g_hover_innov_var = 0.0f;
    g_hover_test_ratio = 0.0f;
    g_hover_meas_acc = 0.0f;
    g_hover_meas_noise = 0.0f;
    g_hover_status = 0U;
}

static void control_update_hover_thrust_bias_comp (float hover_old, float hover_new, float vel_i_lim)
{
    float delta = 0.0f;

    if (fabsf(hover_new - hover_old) < 1e-4f)
    {
        return;
    }

    delta = hover_old - hover_new;
    g_alt_i_state = task_clamp_float(g_alt_i_state + delta, -vel_i_lim, vel_i_lim);
}

static void control_hover_thrust_estimator_step (uint8 enable,
                                                 uint8 armed,
                                                 uint8 estimator_healthy,
                                                 uint8 altitude_control_active,
                                                 uint8 landing_settled,
                                                 uint8 thrust_high_limited,
                                                 uint8 thrust_low_limited,
                                                 float hover_base,
                                                 float hover_range,
                                                 float hover_noise,
                                                 float hover_init_std,
                                                 float accel_gate,
                                                 float vxy_thr,
                                                 float vz_thr,
                                                 float limit_thr_min,
                                                 float limit_thr_max,
                                                 float thrust_cmd,
                                                 float vel_x,
                                                 float vel_y,
                                                 float vel_z,
                                                 float vel_i_lim,
                                                 float dt,
                                                 uint32 now_ms)
{
    float old_hover = 0.0f;
    float min_hover = 0.0f;
    float max_hover = 0.0f;
    float accel_meas = 0.0f;
    float meas_noise = 0.0f;
    float h = 0.0f;
    float innov = 0.0f;
    float innov_var = 0.0f;
    float gate_sq = 0.0f;
    float test_ratio = 0.0f;
    float k_gain = 0.0f;
    float speed_xy = 0.0f;
    float noise_scale_xy = 0.0f;
    float noise_scale_z = 0.0f;
    float valid_var_threshold = 0.0f;
    float accept_ratio = 0.0f;

    if (!g_hover_est_initialized)
    {
        control_reset_hover_thrust_estimator(hover_base, hover_init_std);
    }

    g_hover_status = 0U;
    if (!enable)
    {
        control_reset_hover_thrust_estimator(hover_base, hover_init_std);
        return;
    }
    g_hover_status |= HTE_STATUS_ENABLED;

    if ((!armed) || (!estimator_healthy))
    {
        control_reset_hover_thrust_estimator(hover_base, hover_init_std);
        g_hover_status |= HTE_STATUS_ENABLED;
        return;
    }

    if (dt <= 0.0005f)
    {
        return;
    }

    accel_meas = (vel_z - g_prev_vz_est) / dt;
    g_prev_vz_est = vel_z;
    g_acc_z_lpf += task_clamp_float((accel_meas - g_acc_z_lpf) * dt * 6.0f, -5.0f, 5.0f);
    g_hover_meas_acc = g_acc_z_lpf;

    if ((g_hover_last_fuse_ms != 0U) && ((uint32)(now_ms - g_hover_last_fuse_ms) > HTE_VALID_STALE_MS))
    {
        g_hover_est_valid = 0U;
        g_hover_status |= HTE_STATUS_STALE;
    }

    if ((!altitude_control_active) || landing_settled || thrust_high_limited || thrust_low_limited)
    {
        return;
    }

    if ((thrust_cmd <= (limit_thr_min + 25.0f)) || (thrust_cmd >= (limit_thr_max - 25.0f)))
    {
        return;
    }
    g_hover_status |= HTE_STATUS_CONDITIONS_OK;

    if ((g_hover_window_start_ms == 0U) || ((uint32)(now_ms - g_hover_window_start_ms) > HTE_VALID_WINDOW_MS))
    {
        g_hover_window_start_ms = now_ms;
        g_hover_window_attempts = 0U;
        g_hover_window_accepts = 0U;
        g_hover_window_rejects = 0U;
    }

    min_hover = hover_base - hover_range;
    max_hover = hover_base + hover_range;
    if (min_hover < (limit_thr_min + 20.0f))
    {
        min_hover = limit_thr_min + 20.0f;
    }
    if (max_hover > (limit_thr_max - 20.0f))
    {
        max_hover = limit_thr_max - 20.0f;
    }
    if (min_hover > max_hover)
    {
        min_hover = hover_base;
        max_hover = hover_base;
    }

    speed_xy = sqrtf(vel_x * vel_x + vel_y * vel_y);
    noise_scale_xy = (speed_xy > vxy_thr) ? (1.0f + (speed_xy - vxy_thr)) : 1.0f;
    noise_scale_z = (fabsf(vel_z) > vz_thr) ? (1.0f + (fabsf(vel_z) - vz_thr)) : 1.0f;
    meas_noise = 1.5f * ((noise_scale_xy > noise_scale_z) ? noise_scale_xy : noise_scale_z);
    if (meas_noise < 0.4f)
    {
        meas_noise = 0.4f;
    }
    g_hover_meas_noise = meas_noise;

    if (hover_noise < 0.1f)
    {
        hover_noise = 0.1f;
    }
    g_hover_thrust_var += dt * hover_noise * hover_noise;

    if (g_hover_thrust_est < 1.0f)
    {
        g_hover_thrust_est = hover_base;
    }

    h = -GRAVITY_MSS * thrust_cmd / (g_hover_thrust_est * g_hover_thrust_est);
    innov = g_acc_z_lpf - GRAVITY_MSS * (thrust_cmd / g_hover_thrust_est - 1.0f);
    innov_var = h * h * g_hover_thrust_var + meas_noise * meas_noise;
    if (innov_var < 1e-4f)
    {
        innov_var = 1e-4f;
    }
    g_hover_innov = innov;
    g_hover_innov_var = innov_var;

    gate_sq = accel_gate * accel_gate;
    if (gate_sq < 1.0f)
    {
        gate_sq = 1.0f;
    }
    test_ratio = (innov * innov) / (innov_var * gate_sq);
    g_hover_test_ratio = test_ratio;
    if (g_hover_window_attempts < 65535U)
    {
        g_hover_window_attempts++;
    }
    if (test_ratio > 1.0f)
    {
        g_hover_status |= HTE_STATUS_INNOV_REJECTED;
        if (g_hover_window_rejects < 65535U)
        {
            g_hover_window_rejects++;
        }
        g_hover_est_valid = 0U;
        return;
    }

    old_hover = g_hover_thrust_est;
    k_gain = g_hover_thrust_var * h / innov_var;
    g_hover_thrust_est = task_clamp_float(g_hover_thrust_est + k_gain * innov, min_hover, max_hover);
    g_hover_thrust_var = task_clamp_float(g_hover_thrust_var - k_gain * h * g_hover_thrust_var,
                                          1.0f,
                                          hover_init_std * hover_init_std);
    control_update_hover_thrust_bias_comp(old_hover, g_hover_thrust_est, vel_i_lim);
    g_hover_last_fuse_ms = now_ms;
    g_hover_status |= HTE_STATUS_FUSED;
    if (g_hover_window_accepts < 65535U)
    {
        g_hover_window_accepts++;
    }

    valid_var_threshold = 0.08f * hover_init_std * hover_init_std;
    if (valid_var_threshold < 25.0f)
    {
        valid_var_threshold = 25.0f;
    }
    if (g_hover_window_attempts > 0U)
    {
        accept_ratio = (float)g_hover_window_accepts / (float)g_hover_window_attempts;
    }
    g_hover_est_valid = ((g_hover_window_attempts >= HTE_VALID_MIN_SAMPLES) &&
                         (g_hover_window_accepts >= HTE_VALID_MIN_SAMPLES) &&
                         (accept_ratio >= HTE_VALID_MIN_ACCEPT_RATIO) &&
                         (g_hover_thrust_var <= valid_var_threshold) &&
                         ((uint32)(now_ms - g_hover_last_fuse_ms) <= HTE_VALID_STALE_MS)) ? 1U : 0U;
    if (g_hover_est_valid)
    {
        g_hover_status |= HTE_STATUS_VALID;
    }
}

static float control_rate_pid_step (float rate_error,
                                    float *rate_i_state,
                                    float *rate_error_prev,
                                    float kp,
                                    float ki,
                                    float kd,
                                    float int_lim,
                                    uint8 mixer_limited,
                                    float dt)
{
    float i_candidate = 0.0f;
    float deriv = 0.0f;
    float u_unsat = 0.0f;
    float u_sat = 0.0f;
    uint8 hold_integrator = 0U;

    i_candidate = *rate_i_state + ki * rate_error * dt;
    i_candidate = task_clamp_float(i_candidate, -int_lim, int_lim);
    deriv = (rate_error - *rate_error_prev) / dt;
    u_unsat = kp * rate_error + i_candidate + kd * deriv;
    u_sat = task_clamp_float(u_unsat, -1.0f, 1.0f);

    if ((u_unsat > 1.0f && rate_error > 0.0f) || (u_unsat < -1.0f && rate_error < 0.0f))
    {
        hold_integrator = 1U;
    }

    if (mixer_limited && ((u_unsat * rate_error) > 1e-5f))
    {
        hold_integrator = 1U;
    }

    if (!hold_integrator)
    {
        *rate_i_state = i_candidate;
    }

    *rate_error_prev = rate_error;
    return u_sat;
}

static float control_altitude_to_velocity_sp (float altitude_sp,
                                              float altitude_meas,
                                              float altitude_p,
                                              float vel_up_max,
                                              float vel_down_max)
{
    float altitude_error = altitude_sp - altitude_meas;
    return task_clamp_float(altitude_error * altitude_p, -vel_down_max, vel_up_max);
}

static float control_vertical_velocity_pid_step (float vel_sp,
                                                 float vel_meas,
                                                 float hover_throttle,
                                                 float vel_kp,
                                                 float vel_ki,
                                                 float vel_kd,
                                                 float vel_i_lim,
                                                 float limit_thr_min,
                                                 float limit_thr_max,
                                                 uint8 thrust_high_limited,
                                                 uint8 thrust_low_limited,
                                                 float dt)
{
    float vel_error = vel_sp - vel_meas;
    float i_candidate = g_alt_i_state + vel_ki * vel_error * dt;
    float deriv = (vel_error - g_prev_vel_z_error) / dt;
    float u_unsat = 0.0f;
    float u_sat = 0.0f;
    uint8 hold_integrator = 0U;

    i_candidate = task_clamp_float(i_candidate, -vel_i_lim, vel_i_lim);
    u_unsat = hover_throttle + vel_error * vel_kp + i_candidate + vel_kd * deriv;
    u_sat = task_clamp_float(u_unsat, limit_thr_min, limit_thr_max);

    if ((u_unsat > limit_thr_max && vel_error > 0.0f) ||
        (u_unsat < limit_thr_min && vel_error < 0.0f))
    {
        hold_integrator = 1U;
    }

    if (thrust_high_limited && (vel_error > 0.0f))
    {
        hold_integrator = 1U;
    }
    if (thrust_low_limited && (vel_error < 0.0f))
    {
        hold_integrator = 1U;
    }

    if (!hold_integrator)
    {
        g_alt_i_state = i_candidate;
    }

    g_prev_vel_z_error = vel_error;
    return u_sat;
}

static void control_task_entry (void *parameter)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_tick = last_wake;

    (void)parameter;

    while (1)
    {
        shm_commander_data_struct commander = {0};
        shm_estimator_data_struct estimator = {0};
        shm_host_control_data_struct host = {0};
        shm_mixer_data_struct mixer = {0};
        shm_control_data_struct out = {0};
        TickType_t now_tick = xTaskGetTickCount();
        uint32 now_ms = task_now_ms();
        float dt = (float)(now_tick - last_tick) * 0.001f;
        float alt_target = 0.0f;
        float alt_z_p = 0.0f;
        float thr_hover = 0.0f;
        float vel_z_p = 0.0f;
        float vel_z_i = 0.0f;
        float vel_z_d = 0.0f;
        float vel_z_i_lim = 0.0f;
        float vel_z_max_up = 0.0f;
        float vel_z_max_dn = 0.0f;
        float hte_hover_noise = 0.0f;
        float hte_hover_init_std = 0.0f;
        float hte_acc_gate = 0.0f;
        float hte_thr_range = 0.0f;
        float hte_vxy_thr = 0.0f;
        float hte_vz_thr = 0.0f;
        float att_roll_p = 0.0f;
        float att_pitch_p = 0.0f;
        float att_yaw_p = 0.0f;
        float rate_roll_p = 0.0f;
        float rate_roll_i = 0.0f;
        float rate_roll_d = 0.0f;
        float rate_pitch_p = 0.0f;
        float rate_pitch_i = 0.0f;
        float rate_pitch_d = 0.0f;
        float rate_yaw_p = 0.0f;
        float rate_yaw_i = 0.0f;
        float rate_yaw_d = 0.0f;
        float rate_int_lim_rp = 0.0f;
        float rate_int_lim_yaw = 0.0f;
        float rate_max_roll_dps = 0.0f;
        float rate_max_pitch_dps = 0.0f;
        float rate_max_yaw_dps = 0.0f;
        float pos_xy_p = 0.0f;
        float vel_xy_p = 0.0f;
        float pos_vel_max_mps = 0.0f;
        float pos_tilt_max_deg = 0.0f;
        float host_timeout_s = 0.0f;
        float host_xy_max_mps = 0.0f;
        float host_z_max_mps = 0.0f;
        float host_yaw_max_dps = 0.0f;
        float host_thr_span = 0.0f;
        float land_descend_mps = 0.0f;
        float limit_roll = 0.0f;
        float limit_pitch = 0.0f;
        float limit_yaw = 0.0f;
        float limit_thr_min = 0.0f;
        float limit_thr_max = 0.0f;
        float rate_sp_roll = 0.0f;
        float rate_sp_pitch = 0.0f;
        float rate_sp_yaw = 0.0f;
        float rate_meas_roll = 0.0f;
        float rate_meas_pitch = 0.0f;
        float rate_meas_yaw = 0.0f;
        float roll_sp = 0.0f;
        float pitch_sp = 0.0f;
        float vel_sp_x = 0.0f;
        float vel_sp_y = 0.0f;
        float vel_sp_z = 0.0f;
        float altitude_sp = 0.0f;
        float roll_norm = 0.0f;
        float pitch_norm = 0.0f;
        float yaw_norm = 0.0f;
        uint8 flight_mode = 0U;
        uint8 nav_state = 0U;
        uint8 land_state = 0U;
        uint8 poshold_active = 0U;
        uint8 host_ctrl_enable = 0U;
        uint8 host_control_ok = 0U;
        uint8 manual_throttle_mode = 0U;
        uint8 hover_est_enable = 0U;
        uint8 altitude_control_active = 0U;
        uint8 landing_contact = 0U;
        uint8 landing_settled = 0U;
        uint8 mixer_limit_rp = 0U;
        uint8 mixer_limit_yaw = 0U;
        uint8 thrust_high_limited = 0U;
        uint8 thrust_low_limited = 0U;
        uint8 rate_valid = 0U;
        uint32 host_timeout_ms = 0U;

        if (dt <= 0.0005f)
        {
            dt = (float)TASK_CONTROL_PERIOD_MS * 0.001f;
        }
        if (dt > 0.02f)
        {
            dt = 0.02f;
        }
        last_tick = now_tick;

        (void)shm_read_commander(&commander, 0);
        (void)shm_read_estimator(&estimator, 0);
        (void)shm_read_host_control(&host, 0);
        (void)shm_read_mixer(&mixer, 0);

        alt_target = flight_param_get_f32_default(FLIGHT_PARAM_ALT_TARGET_M, 1.0f);
        alt_z_p = flight_param_get_f32_default(FLIGHT_PARAM_ALT_Z_P, 1.0f);
        thr_hover = flight_param_get_f32_default(FLIGHT_PARAM_THR_BASE, 350.0f);
        vel_z_p = flight_param_get_f32_default(FLIGHT_PARAM_THR_ALT_KP, 180.0f);
        vel_z_i = flight_param_get_f32_default(FLIGHT_PARAM_THR_ALT_I, 60.0f);
        vel_z_d = flight_param_get_f32_default(FLIGHT_PARAM_VEL_Z_D, 30.0f);
        vel_z_i_lim = flight_param_get_f32_default(FLIGHT_PARAM_THR_ALT_I_LIM, 180.0f);
        vel_z_max_up = flight_param_get_f32_default(FLIGHT_PARAM_VEL_Z_MAX_UP, 1.5f);
        vel_z_max_dn = flight_param_get_f32_default(FLIGHT_PARAM_VEL_Z_MAX_DN, 0.8f);
        hover_est_enable = flight_param_get_u8_default(FLIGHT_PARAM_HTE_ENABLE, 1U);
        hte_hover_noise = flight_param_get_f32_default(FLIGHT_PARAM_HTE_HT_NOISE, 6.0f);
        hte_hover_init_std = flight_param_get_f32_default(FLIGHT_PARAM_HTE_HT_ERR_INIT, 80.0f);
        hte_acc_gate = flight_param_get_f32_default(FLIGHT_PARAM_HTE_ACC_GATE, 3.0f);
        hte_thr_range = flight_param_get_f32_default(FLIGHT_PARAM_HTE_THR_RANGE, 120.0f);
        hte_vxy_thr = flight_param_get_f32_default(FLIGHT_PARAM_HTE_VXY_THR, 1.5f);
        hte_vz_thr = flight_param_get_f32_default(FLIGHT_PARAM_HTE_VZ_THR, 0.8f);
        att_roll_p = flight_param_get_f32_default(FLIGHT_PARAM_ROLL_KP, 4.0f);
        att_pitch_p = flight_param_get_f32_default(FLIGHT_PARAM_PITCH_KP, 4.0f);
        att_yaw_p = flight_param_get_f32_default(FLIGHT_PARAM_YAW_KP, 2.8f);
        limit_roll = flight_param_get_f32_default(FLIGHT_PARAM_OUT_LIMIT_ROLL, 220.0f);
        limit_pitch = flight_param_get_f32_default(FLIGHT_PARAM_OUT_LIMIT_PITCH, 220.0f);
        limit_yaw = flight_param_get_f32_default(FLIGHT_PARAM_OUT_LIMIT_YAW, 160.0f);
        limit_thr_min = flight_param_get_f32_default(FLIGHT_PARAM_OUT_LIMIT_THR_MIN, 120.0f);
        limit_thr_max = flight_param_get_f32_default(FLIGHT_PARAM_OUT_LIMIT_THR_MAX, 900.0f);
        rate_roll_p = flight_param_get_f32_default(FLIGHT_PARAM_RATE_ROLL_P, 0.15f);
        rate_roll_i = flight_param_get_f32_default(FLIGHT_PARAM_RATE_ROLL_I, 0.20f);
        rate_roll_d = flight_param_get_f32_default(FLIGHT_PARAM_RATE_ROLL_D, 0.003f);
        rate_pitch_p = flight_param_get_f32_default(FLIGHT_PARAM_RATE_PITCH_P, 0.15f);
        rate_pitch_i = flight_param_get_f32_default(FLIGHT_PARAM_RATE_PITCH_I, 0.20f);
        rate_pitch_d = flight_param_get_f32_default(FLIGHT_PARAM_RATE_PITCH_D, 0.003f);
        rate_yaw_p = flight_param_get_f32_default(FLIGHT_PARAM_RATE_YAW_P, 0.20f);
        rate_yaw_i = flight_param_get_f32_default(FLIGHT_PARAM_RATE_YAW_I, 0.10f);
        rate_yaw_d = flight_param_get_f32_default(FLIGHT_PARAM_RATE_YAW_D, 0.0f);
        rate_int_lim_rp = flight_param_get_f32_default(FLIGHT_PARAM_RATE_INT_LIM_RP, 0.30f);
        rate_int_lim_yaw = flight_param_get_f32_default(FLIGHT_PARAM_RATE_INT_LIM_YAW, 0.30f);
        rate_max_roll_dps = flight_param_get_f32_default(FLIGHT_PARAM_RATE_MAX_ROLL_DPS, 220.0f);
        rate_max_pitch_dps = flight_param_get_f32_default(FLIGHT_PARAM_RATE_MAX_PITCH_DPS, 220.0f);
        rate_max_yaw_dps = flight_param_get_f32_default(FLIGHT_PARAM_RATE_MAX_YAW_DPS, 200.0f);
        pos_xy_p = flight_param_get_f32_default(FLIGHT_PARAM_POS_XY_P, 0.8f);
        vel_xy_p = flight_param_get_f32_default(FLIGHT_PARAM_VEL_XY_P, 0.35f);
        pos_vel_max_mps = flight_param_get_f32_default(FLIGHT_PARAM_POS_VEL_MAX_MPS, 2.0f);
        pos_tilt_max_deg = flight_param_get_f32_default(FLIGHT_PARAM_POS_TILT_MAX_DEG, 20.0f);
        host_ctrl_enable = flight_param_get_u8_default(FLIGHT_PARAM_HOST_CTRL_ENABLE, 0U);
        host_timeout_s = flight_param_get_f32_default(FLIGHT_PARAM_HOST_TIMEOUT_S, 0.35f);
        host_xy_max_mps = flight_param_get_f32_default(FLIGHT_PARAM_HOST_XY_MAX_MPS, 1.0f);
        host_z_max_mps = flight_param_get_f32_default(FLIGHT_PARAM_HOST_Z_MAX_MPS, 0.8f);
        host_yaw_max_dps = flight_param_get_f32_default(FLIGHT_PARAM_HOST_YAW_MAX_DPS, 90.0f);
        host_thr_span = flight_param_get_f32_default(FLIGHT_PARAM_HOST_THR_SPAN, 180.0f);
        land_descend_mps = flight_param_get_f32_default(FLIGHT_PARAM_LAND_DESCEND_MPS, 0.30f);
        host_timeout_ms = (uint32)(host_timeout_s * 1000.0f);
        if (host_timeout_ms < 50U)
        {
            host_timeout_ms = 50U;
        }

        host_control_ok = (host_ctrl_enable &&
                           host.control_active &&
                           control_host_recent(&host, now_ms, host_timeout_ms)) ? 1U : 0U;
        thrust_high_limited = (mixer.saturation_high || mixer.thrust_reduced) ? 1U : 0U;
        thrust_low_limited = mixer.saturation_low ? 1U : 0U;
        mixer_limit_rp = (mixer.saturation_high || mixer.saturation_low || mixer.thrust_reduced) ? 1U : 0U;
        mixer_limit_yaw = (mixer.yaw_limited || mixer.saturation_high || mixer.saturation_low) ? 1U : 0U;
        flight_mode = commander.flight_mode;
        nav_state = commander.nav_state;
        land_state = commander.land_state;
        landing_contact = ((nav_state == SHM_NAV_STATE_LAND) && (land_state >= SHM_LAND_STATE_GROUND_CONTACT)) ? 1U : 0U;
        landing_settled = ((nav_state == SHM_NAV_STATE_LAND) && (land_state >= SHM_LAND_STATE_MAYBE_LANDED)) ? 1U : 0U;
        if (flight_mode > FLIGHT_MODE_POSHOLD_FLOW_TOF)
        {
            flight_mode = FLIGHT_MODE_STABILIZE_TEST;
        }

        out.timestamp_ms = now_ms;
        out.armed = commander.armed;
        out.failsafe = commander.failsafe;
        out.telemetry_flags = 0U;

        if (!g_hover_est_initialized)
        {
            control_reset_hover_thrust_estimator(thr_hover, hte_hover_init_std);
        }

        if (commander.armed && !commander.failsafe && estimator.healthy)
        {
            if ((flight_mode == FLIGHT_MODE_POSHOLD_FLOW_TOF) && estimator.flow_fused)
            {
                poshold_active = 1U;
            }

            if (nav_state != SHM_NAV_STATE_NONE)
            {
                if (!g_hold_sp_valid)
                {
                    g_hold_x_sp_m = estimator.pos_x_m;
                    g_hold_y_sp_m = estimator.pos_y_m;
                    g_hold_sp_valid = 1U;
                }
                if (!g_host_alt_sp_valid)
                {
                    g_host_alt_sp_m = estimator.altitude_m;
                    g_host_alt_sp_valid = 1U;
                }
                if (!g_host_yaw_sp_valid)
                {
                    g_host_yaw_sp_rad = task_wrap_pi(estimator.yaw_rad);
                    g_host_yaw_sp_valid = 1U;
                }
                if (nav_state == SHM_NAV_STATE_LAND)
                {
                    g_host_alt_sp_m -= land_descend_mps * dt;
                    g_host_alt_sp_m = task_clamp_float(g_host_alt_sp_m, 0.0f, 5.0f);
                }

                if (landing_contact)
                {
                    g_hold_x_sp_m = estimator.pos_x_m;
                    g_hold_y_sp_m = estimator.pos_y_m;
                    vel_sp_x = 0.0f;
                    vel_sp_y = 0.0f;
                    roll_sp = 0.0f;
                    pitch_sp = 0.0f;
                }
                else if (estimator.flow_fused)
                {
                    vel_sp_x = task_clamp_float((g_hold_x_sp_m - estimator.pos_x_m) * pos_xy_p,
                                                -pos_vel_max_mps,
                                                pos_vel_max_mps);
                    vel_sp_y = task_clamp_float((g_hold_y_sp_m - estimator.pos_y_m) * pos_xy_p,
                                                -pos_vel_max_mps,
                                                pos_vel_max_mps);

                    pitch_sp = task_clamp_float((vel_sp_x - estimator.vel_x_mps) * vel_xy_p,
                                                -pos_tilt_max_deg * DEG2RAD,
                                                pos_tilt_max_deg * DEG2RAD);
                    roll_sp = task_clamp_float(-(vel_sp_y - estimator.vel_y_mps) * vel_xy_p,
                                               -pos_tilt_max_deg * DEG2RAD,
                                               pos_tilt_max_deg * DEG2RAD);
                }
                altitude_sp = g_host_alt_sp_m;
            }
            else if (poshold_active)
            {
                if (!g_hold_sp_valid)
                {
                    g_hold_x_sp_m = estimator.pos_x_m;
                    g_hold_y_sp_m = estimator.pos_y_m;
                    g_hold_sp_valid = 1U;
                }
                if (!g_host_alt_sp_valid)
                {
                    g_host_alt_sp_m = estimator.altitude_m;
                    g_host_alt_sp_valid = 1U;
                }
                if (!g_host_yaw_sp_valid)
                {
                    g_host_yaw_sp_rad = task_wrap_pi(estimator.yaw_rad);
                    g_host_yaw_sp_valid = 1U;
                }

                if (host_control_ok)
                {
                    g_hold_x_sp_m += task_clamp_float(host.forward, -1.0f, 1.0f) * host_xy_max_mps * dt;
                    g_hold_y_sp_m += task_clamp_float(host.right, -1.0f, 1.0f) * host_xy_max_mps * dt;
                    g_host_alt_sp_m += task_clamp_float(host.up, -1.0f, 1.0f) * host_z_max_mps * dt;
                    g_host_alt_sp_m = task_clamp_float(g_host_alt_sp_m, 0.1f, 5.0f);
                    g_host_yaw_sp_rad = task_wrap_pi(g_host_yaw_sp_rad +
                                                     task_clamp_float(host.yaw, -1.0f, 1.0f) *
                                                     host_yaw_max_dps * DEG2RAD * dt);
                }

                vel_sp_x = task_clamp_float((g_hold_x_sp_m - estimator.pos_x_m) * pos_xy_p,
                                            -pos_vel_max_mps,
                                            pos_vel_max_mps);
                vel_sp_y = task_clamp_float((g_hold_y_sp_m - estimator.pos_y_m) * pos_xy_p,
                                            -pos_vel_max_mps,
                                            pos_vel_max_mps);

                pitch_sp = task_clamp_float((vel_sp_x - estimator.vel_x_mps) * vel_xy_p,
                                            -pos_tilt_max_deg * DEG2RAD,
                                            pos_tilt_max_deg * DEG2RAD);
                roll_sp = task_clamp_float(-(vel_sp_y - estimator.vel_y_mps) * vel_xy_p,
                                           -pos_tilt_max_deg * DEG2RAD,
                                           pos_tilt_max_deg * DEG2RAD);
                altitude_sp = g_host_alt_sp_m;
            }
            else
            {
                g_hold_sp_valid = 0U;
                g_host_alt_sp_valid = 0U;
                g_host_yaw_sp_valid = 0U;

                if (host_control_ok)
                {
                    roll_sp = task_clamp_float(-host.right, -1.0f, 1.0f) * pos_tilt_max_deg * DEG2RAD;
                    pitch_sp = task_clamp_float(host.forward, -1.0f, 1.0f) * pos_tilt_max_deg * DEG2RAD;
                }
                altitude_sp = alt_target;
            }

            rate_sp_roll = task_clamp_float((roll_sp - estimator.roll_rad) * att_roll_p,
                                            -rate_max_roll_dps * DEG2RAD,
                                            rate_max_roll_dps * DEG2RAD);
            rate_sp_pitch = task_clamp_float((pitch_sp - estimator.pitch_rad) * att_pitch_p,
                                             -rate_max_pitch_dps * DEG2RAD,
                                             rate_max_pitch_dps * DEG2RAD);

            if ((nav_state != SHM_NAV_STATE_NONE) || poshold_active)
            {
                rate_sp_yaw = task_clamp_float(task_wrap_pi(g_host_yaw_sp_rad - estimator.yaw_rad) * att_yaw_p,
                                               -rate_max_yaw_dps * DEG2RAD,
                                               rate_max_yaw_dps * DEG2RAD);
            }
            else if (host_control_ok)
            {
                rate_sp_yaw = task_clamp_float(task_clamp_float(host.yaw, -1.0f, 1.0f) * host_yaw_max_dps * DEG2RAD,
                                               -rate_max_yaw_dps * DEG2RAD,
                                               rate_max_yaw_dps * DEG2RAD);
            }
            else
            {
                rate_sp_yaw = task_clamp_float(-task_wrap_pi(estimator.yaw_rad) * att_yaw_p,
                                               -rate_max_yaw_dps * DEG2RAD,
                                               rate_max_yaw_dps * DEG2RAD);
            }

            if (landing_contact)
            {
                rate_sp_yaw = 0.0f;
            }

            rate_valid = control_get_body_rates_rps(&rate_meas_roll, &rate_meas_pitch, &rate_meas_yaw, now_ms);

            manual_throttle_mode = ((nav_state == SHM_NAV_STATE_NONE) && (!poshold_active) && host_control_ok) ? 1U : 0U;
            if (manual_throttle_mode)
            {
                control_reset_altitude_state();
                out.telemetry_flags |= CONTROL_TELEM_FLAG_MANUAL_THR;
                out.throttle = task_clamp_float(thr_hover +
                                                task_clamp_float(host.up, -1.0f, 1.0f) * host_thr_span,
                                                limit_thr_min,
                                                limit_thr_max);
            }
            else
            {
                altitude_control_active = 1U;
                if (landing_settled)
                {
                    control_reset_altitude_state();
                    vel_sp_z = 0.0f;
                    out.throttle = limit_thr_min;
                }
                else
                {
                    vel_sp_z = control_altitude_to_velocity_sp(altitude_sp,
                                                               estimator.altitude_m,
                                                               alt_z_p,
                                                               vel_z_max_up,
                                                               vel_z_max_dn);
                    out.throttle = control_vertical_velocity_pid_step(vel_sp_z,
                                                                      estimator.vertical_speed_mps,
                                                                      g_hover_thrust_est,
                                                                      vel_z_p,
                                                                      vel_z_i,
                                                                      vel_z_d,
                                                                      vel_z_i_lim,
                                                                      limit_thr_min,
                                                                      limit_thr_max,
                                                                      thrust_high_limited,
                                                                      thrust_low_limited,
                                                                      dt);
                    if (thrust_high_limited)
                    {
                        out.telemetry_flags |= CONTROL_TELEM_FLAG_ALT_AW_HIGH;
                    }
                    if (thrust_low_limited)
                    {
                        out.telemetry_flags |= CONTROL_TELEM_FLAG_ALT_AW_LOW;
                    }
                }
            }

            control_hover_thrust_estimator_step(hover_est_enable,
                                                commander.armed,
                                                estimator.healthy,
                                                altitude_control_active,
                                                landing_settled,
                                                thrust_high_limited,
                                                thrust_low_limited,
                                                thr_hover,
                                                hte_thr_range,
                                                hte_hover_noise,
                                                hte_hover_init_std,
                                                hte_acc_gate,
                                                hte_vxy_thr,
                                                hte_vz_thr,
                                                limit_thr_min,
                                                limit_thr_max,
                                                out.throttle,
                                                estimator.vel_x_mps,
                                                estimator.vel_y_mps,
                                                estimator.vertical_speed_mps,
                                                vel_z_i_lim,
                                                dt,
                                                now_ms);
            if (g_hover_est_valid)
            {
                out.telemetry_flags |= CONTROL_TELEM_FLAG_HTE_VALID;
            }

            if (rate_valid)
            {
                if (landing_contact)
                {
                    control_reset_rate_pid_state();
                    out.roll_cmd = 0.0f;
                    out.pitch_cmd = 0.0f;
                    out.yaw_cmd = 0.0f;
                }
                else
                {
                    roll_norm = control_rate_pid_step(rate_sp_roll - rate_meas_roll,
                                                      &g_rate_i_roll,
                                                      &g_prev_rate_err_roll,
                                                      rate_roll_p,
                                                      rate_roll_i,
                                                      rate_roll_d,
                                                      rate_int_lim_rp,
                                                      mixer_limit_rp,
                                                      dt);
                    pitch_norm = control_rate_pid_step(rate_sp_pitch - rate_meas_pitch,
                                                       &g_rate_i_pitch,
                                                       &g_prev_rate_err_pitch,
                                                       rate_pitch_p,
                                                       rate_pitch_i,
                                                       rate_pitch_d,
                                                       rate_int_lim_rp,
                                                       mixer_limit_rp,
                                                       dt);
                    yaw_norm = control_rate_pid_step(rate_sp_yaw - rate_meas_yaw,
                                                     &g_rate_i_yaw,
                                                     &g_prev_rate_err_yaw,
                                                     rate_yaw_p,
                                                     rate_yaw_i,
                                                     rate_yaw_d,
                                                     rate_int_lim_yaw,
                                                     mixer_limit_yaw,
                                                     dt);

                    out.roll_cmd = task_clamp_float(roll_norm * limit_roll, -limit_roll, limit_roll);
                    out.pitch_cmd = task_clamp_float(pitch_norm * limit_pitch, -limit_pitch, limit_pitch);
                    out.yaw_cmd = task_clamp_float(yaw_norm * limit_yaw, -limit_yaw, limit_yaw);

                    if (mixer_limit_rp)
                    {
                        out.telemetry_flags |= CONTROL_TELEM_FLAG_RATE_AW_RP;
                    }
                    if (mixer_limit_yaw)
                    {
                        out.telemetry_flags |= CONTROL_TELEM_FLAG_RATE_AW_YAW;
                    }
                }
            }
            else
            {
                control_reset_rate_pid_state();
            }

            out.altitude_sp_m = altitude_sp;
            out.vel_sp_x_mps = vel_sp_x;
            out.vel_sp_y_mps = vel_sp_y;
            out.vel_sp_z_mps = vel_sp_z;
            out.alt_i_term = g_alt_i_state;
            out.roll_sp_rad = roll_sp;
            out.pitch_sp_rad = pitch_sp;
            out.rate_sp_roll_rps = rate_sp_roll;
            out.rate_sp_pitch_rps = rate_sp_pitch;
            out.rate_sp_yaw_rps = rate_sp_yaw;
            out.rate_meas_roll_rps = rate_meas_roll;
            out.rate_meas_pitch_rps = rate_meas_pitch;
            out.rate_meas_yaw_rps = rate_meas_yaw;
            out.hover_thrust_status = g_hover_status;
            out.hover_thrust_valid = g_hover_est_valid;
            out.hover_thrust_est = g_hover_thrust_est;
            out.hover_thrust_var = g_hover_thrust_var;
            out.hover_thrust_innov = g_hover_innov;
            out.hover_thrust_innov_var = g_hover_innov_var;
            out.hover_thrust_test_ratio = g_hover_test_ratio;
            out.hover_thrust_accel_mps2 = g_hover_meas_acc;
            out.hover_thrust_accel_noise = g_hover_meas_noise;
        }
        else
        {
            control_reset_rate_pid_state();
            control_reset_altitude_state();
            control_reset_hover_thrust_estimator(thr_hover, hte_hover_init_std);
            g_hold_sp_valid = 0U;
            g_host_alt_sp_valid = 0U;
            g_host_yaw_sp_valid = 0U;
            out.throttle = 0.0f;
            out.roll_cmd = 0.0f;
            out.pitch_cmd = 0.0f;
            out.yaw_cmd = 0.0f;
            out.hover_thrust_status = g_hover_status;
            out.hover_thrust_valid = g_hover_est_valid;
            out.hover_thrust_est = g_hover_thrust_est;
            out.hover_thrust_var = g_hover_thrust_var;
            out.hover_thrust_innov = g_hover_innov;
            out.hover_thrust_innov_var = g_hover_innov_var;
            out.hover_thrust_test_ratio = g_hover_test_ratio;
            out.hover_thrust_accel_mps2 = g_hover_meas_acc;
            out.hover_thrust_accel_noise = g_hover_meas_noise;
        }

        shm_publish_control(&out);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS));
    }
}

void tasks_control_init (void)
{
    if (g_task_control == 0)
    {
        xTaskCreate(control_task_entry, TASK_CONTROL_NAME, TASK_CONTROL_STACK, 0, TASK_CONTROL_PRIORITY, &g_task_control);
    }
}
