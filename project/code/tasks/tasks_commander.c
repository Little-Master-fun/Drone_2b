#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config/flight_params.h"
#include "config/shared_memory.h"
#include "tasks.h"

#define TASK_COMMANDER_NAME            "Commander"
#define TASK_COMMANDER_PERIOD_MS       (20U)
#define TASK_COMMANDER_STACK           (384U)
#define TASK_COMMANDER_PRIORITY        (5U)

#define SENSOR_TIMEOUT_MS              (200U)
#define FLIGHT_MODE_STABILIZE_TEST     (0U)
#define FLIGHT_MODE_POSHOLD_FLOW_TOF   (1U)
#define SHM_NAV_STATE_NONE             (0U)
#define SHM_NAV_STATE_HOLD             (1U)
#define SHM_NAV_STATE_LAND             (2U)
#define SHM_FAILSAFE_REASON_NONE       (0U)
#define SHM_FAILSAFE_REASON_MANDATORY  (1U << 0)
#define SHM_FAILSAFE_REASON_HOST_LINK  (1U << 1)
#define SHM_FAILSAFE_REASON_OPTIONAL   (1U << 2)
#define SHM_FAILSAFE_REASON_HOST_LAND  (1U << 3)
#define SHM_FAILSAFE_REASON_HOST_ESTOP (1U << 4)
#define SHM_LAND_STATE_FLYING          (0U)
#define SHM_LAND_STATE_GROUND_CONTACT  (1U)
#define SHM_LAND_STATE_MAYBE_LANDED    (2U)
#define SHM_LAND_STATE_LANDED          (3U)
#define LAND_CONTACT_ALT_M             (0.25f)
#define LAND_CONTACT_VZ_MPS            (0.35f)
#define LAND_CONTACT_VXY_MPS           (0.30f)
#define LAND_CONTACT_THR_RATIO         (0.70f)
#define LAND_MAYBE_THR_RATIO           (0.55f)
#define LAND_CONTACT_TRIGGER_MS        (350U)
#define LAND_MAYBE_TRIGGER_MS          (600U)
#define LAND_DETECT_TRIGGER_MS         (800U)

static TaskHandle_t g_task_commander = 0;
static uint8 g_nav_state = SHM_NAV_STATE_NONE;
static uint32 g_nav_state_since_ms = 0U;
static uint8 g_host_control_owner = 0U;
static uint8 g_host_arm_release_required = 0U;
static uint8 g_failsafe_reason = SHM_FAILSAFE_REASON_NONE;
static uint8 g_land_state = SHM_LAND_STATE_FLYING;
static uint32 g_ground_contact_since_ms = 0U;
static uint32 g_maybe_landed_since_ms = 0U;
static uint32 g_landed_since_ms = 0U;

static uint32 task_now_ms (void)
{
    return (uint32)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint8 task_data_recent (uint32 timestamp_ms, uint32 now_ms, uint32 timeout_ms)
{
    return (((uint32)(now_ms - timestamp_ms)) <= timeout_ms) ? 1U : 0U;
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

static uint8 task_host_recent (const shm_host_control_data_struct *host, uint32 now_ms, uint32 timeout_ms)
{
    if ((host == 0) || (!host->link_active))
    {
        return 0U;
    }

    return task_data_recent(host->timestamp_ms, now_ms, timeout_ms);
}

static void commander_set_nav_state (uint8 nav_state, uint32 now_ms)
{
    if (g_nav_state != nav_state)
    {
        g_nav_state = nav_state;
        g_nav_state_since_ms = now_ms;
    }
}

static void commander_reset_land_detector (void)
{
    g_land_state = SHM_LAND_STATE_FLYING;
    g_ground_contact_since_ms = 0U;
    g_maybe_landed_since_ms = 0U;
    g_landed_since_ms = 0U;
}

static uint8 commander_update_hysteresis (uint8 condition, uint32 *since_ms, uint32 now_ms, uint32 trigger_ms)
{
    if (!condition)
    {
        *since_ms = 0U;
        return 0U;
    }

    if (*since_ms == 0U)
    {
        *since_ms = now_ms;
    }

    return (((uint32)(now_ms - *since_ms)) >= trigger_ms) ? 1U : 0U;
}

static void commander_task_entry (void *parameter)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)parameter;

    while (1)
    {
        shm_imu_data_struct imu = {0};
        shm_flow_data_struct flow = {0};
        shm_tof_data_struct tof = {0};
        shm_comm_data_struct comm = {0};
        shm_host_control_data_struct host = {0};
        shm_estimator_data_struct estimator = {0};
        shm_control_data_struct control = {0};
        shm_commander_data_struct out = {0};
        uint32 now_ms = task_now_ms();
        uint8 flow_enabled = flight_param_get_u8_default(FLIGHT_PARAM_FLOW_ENABLE, 1U);
        uint8 tof_enabled = flight_param_get_u8_default(FLIGHT_PARAM_TOF_ENABLE, 1U);
        uint8 host_ctrl_enable = flight_param_get_u8_default(FLIGHT_PARAM_HOST_CTRL_ENABLE, 0U);
        uint8 host_arm_allow = flight_param_get_u8_default(FLIGHT_PARAM_HOST_ARM_ALLOW, 0U);
        uint8 host_estop_allow = flight_param_get_u8_default(FLIGHT_PARAM_HOST_ESTOP_ALLOW, 1U);
        uint32 host_timeout_ms = (uint32)(flight_param_get_f32_default(FLIGHT_PARAM_HOST_TIMEOUT_S, 0.35f) * 1000.0f);
        uint32 land_hold_ms = (uint32)(flight_param_get_f32_default(FLIGHT_PARAM_LAND_HOLD_S, 3.0f) * 1000.0f);
        float land_disarm_alt_m = flight_param_get_f32_default(FLIGHT_PARAM_LAND_DISARM_ALT_M, 0.15f);
        float thr_hover = flight_param_get_f32_default(FLIGHT_PARAM_THR_BASE, 350.0f);
        float thr_min = flight_param_get_f32_default(FLIGHT_PARAM_OUT_LIMIT_THR_MIN, 120.0f);
        uint8 flight_mode = flight_param_get_u8_default(FLIGHT_PARAM_FLIGHT_MODE, FLIGHT_MODE_STABILIZE_TEST);
        uint8 mandatory_ok = 0U;
        uint8 mode_required_ok = 1U;
        uint8 host_link_ok = 0U;
        uint8 host_armed = 0U;
        uint8 param_armed = 0U;
        uint8 base_armed = 0U;
        uint8 land_trigger = 0U;
        uint8 land_reason = SHM_FAILSAFE_REASON_NONE;
        uint8 optional_sensor_trigger = 0U;
        uint8 hard_failsafe = 0U;
        uint8 control_recent = 0U;
        uint8 low_alt_contact = 0U;
        uint8 low_alt_disarm = 0U;
        uint8 low_vspeed = 0U;
        uint8 low_hspeed = 0U;
        uint8 low_thrust_contact = 0U;
        uint8 low_thrust_maybe = 0U;
        uint8 ground_contact_candidate = 0U;
        uint8 maybe_landed_candidate = 0U;
        uint8 ground_contact = 0U;
        uint8 maybe_landed = 0U;
        uint8 landed = 0U;
        float horizontal_speed_sq = 0.0f;
        float low_thrust_contact_thr = 0.0f;
        float low_thrust_maybe_thr = 0.0f;

        (void)shm_read_imu(0U, &imu, 0);
        (void)shm_read_flow(&flow, 0);
        (void)shm_read_tof(&tof, 0);
        (void)shm_read_comm(&comm, 0);
        (void)shm_read_host_control(&host, 0);
        (void)shm_read_estimator(&estimator, 0);
        (void)shm_read_control(&control, 0);

        if (host_timeout_ms < 50U)
        {
            host_timeout_ms = 50U;
        }
        if (land_hold_ms < 200U)
        {
            land_hold_ms = 200U;
        }

        out.timestamp_ms = now_ms;
        out.flight_mode = (flight_mode <= FLIGHT_MODE_POSHOLD_FLOW_TOF) ? flight_mode : FLIGHT_MODE_STABILIZE_TEST;

        mandatory_ok = (imu.healthy && task_data_recent(imu.timestamp_ms, now_ms, SENSOR_TIMEOUT_MS)) ? 1U : 0U;

        if (out.flight_mode == FLIGHT_MODE_POSHOLD_FLOW_TOF)
        {
            mode_required_ok = ((flow_enabled && tof_enabled) &&
                                flow.healthy &&
                                tof.healthy &&
                                tof.data_ready &&
                                task_data_recent(flow.timestamp_ms, now_ms, SENSOR_TIMEOUT_MS) &&
                                task_data_recent(tof.timestamp_ms, now_ms, SENSOR_TIMEOUT_MS)) ? 1U : 0U;
        }

        out.sensors_healthy = (mandatory_ok && mode_required_ok) ? 1U : 0U;
        host_link_ok = (host_ctrl_enable && comm.link_up && task_host_recent(&host, now_ms, host_timeout_ms)) ? 1U : 0U;

        if (!host.arm_request)
        {
            g_host_arm_release_required = 0U;
        }

        host_armed = (host_link_ok && host_arm_allow && host.arm_request && (!g_host_arm_release_required)) ? 1U : 0U;
        param_armed = flight_param_get_u8_default(FLIGHT_PARAM_ARMING_ENABLE, 0U);
        base_armed = (param_armed || host_armed || (g_nav_state != SHM_NAV_STATE_NONE) || g_host_control_owner) ? 1U : 0U;

        if (host_armed)
        {
            g_host_control_owner = 1U;
        }

        if (host_link_ok && host_estop_allow && host.estop_request)
        {
            commander_set_nav_state(SHM_NAV_STATE_NONE, now_ms);
            commander_reset_land_detector();
            g_failsafe_reason |= SHM_FAILSAFE_REASON_HOST_ESTOP;
            g_host_arm_release_required = 1U;
            g_host_control_owner = 0U;
            (void)flight_param_set_u8(FLIGHT_PARAM_ARMING_ENABLE, 0U);
            host_armed = 0U;
            param_armed = 0U;
            base_armed = 0U;
        }

        optional_sensor_trigger = ((out.flight_mode == FLIGHT_MODE_POSHOLD_FLOW_TOF) && base_armed && (!mode_required_ok)) ? 1U : 0U;
        if (host_link_ok && host.land_request && base_armed)
        {
            land_trigger = 1U;
            land_reason |= SHM_FAILSAFE_REASON_HOST_LAND;
        }
        if (g_host_control_owner && base_armed && (!host_link_ok))
        {
            land_trigger = 1U;
            land_reason |= SHM_FAILSAFE_REASON_HOST_LINK;
        }
        if (optional_sensor_trigger)
        {
            land_trigger = 1U;
            land_reason |= SHM_FAILSAFE_REASON_OPTIONAL;
        }

        if (g_nav_state == SHM_NAV_STATE_NONE)
        {
            if (land_trigger)
            {
                g_failsafe_reason = land_reason;
                commander_set_nav_state(SHM_NAV_STATE_HOLD, now_ms);
            }
        }
        else if (g_nav_state == SHM_NAV_STATE_HOLD)
        {
            if (land_reason != SHM_FAILSAFE_REASON_NONE)
            {
                g_failsafe_reason |= land_reason;
            }
            if (((uint32)(now_ms - g_nav_state_since_ms)) >= land_hold_ms)
            {
                commander_set_nav_state(SHM_NAV_STATE_LAND, now_ms);
            }
        }
        else if (land_reason != SHM_FAILSAFE_REASON_NONE)
        {
            g_failsafe_reason |= land_reason;
        }

        control_recent = task_data_recent(control.timestamp_ms, now_ms, SENSOR_TIMEOUT_MS);
        low_alt_contact = ((tof.healthy && tof.data_ready && (((float)tof.distance_mm * 0.001f) <= LAND_CONTACT_ALT_M)) ||
                           (estimator.healthy && (estimator.altitude_m <= LAND_CONTACT_ALT_M))) ? 1U : 0U;
        low_alt_disarm = ((tof.healthy && tof.data_ready && (((float)tof.distance_mm * 0.001f) <= land_disarm_alt_m)) ||
                          (estimator.healthy && (estimator.altitude_m <= land_disarm_alt_m))) ? 1U : 0U;
        low_vspeed = (estimator.healthy && (fabsf(estimator.vertical_speed_mps) <= LAND_CONTACT_VZ_MPS)) ? 1U : 0U;
        horizontal_speed_sq = (estimator.vel_x_mps * estimator.vel_x_mps) + (estimator.vel_y_mps * estimator.vel_y_mps);
        low_hspeed = (estimator.healthy && (horizontal_speed_sq <= (LAND_CONTACT_VXY_MPS * LAND_CONTACT_VXY_MPS))) ? 1U : 0U;
        low_thrust_contact_thr = task_clamp_float(thr_hover * LAND_CONTACT_THR_RATIO, thr_min, 1200.0f);
        low_thrust_maybe_thr = task_clamp_float(thr_hover * LAND_MAYBE_THR_RATIO, thr_min, 1200.0f);
        low_thrust_contact = (control_recent && (control.throttle <= low_thrust_contact_thr)) ? 1U : 0U;
        low_thrust_maybe = (control_recent && (control.throttle <= low_thrust_maybe_thr)) ? 1U : 0U;

        if ((g_nav_state == SHM_NAV_STATE_LAND) && base_armed)
        {
            ground_contact_candidate = (low_alt_contact && low_vspeed && low_thrust_contact) ? 1U : 0U;
            ground_contact = commander_update_hysteresis(ground_contact_candidate, &g_ground_contact_since_ms, now_ms, LAND_CONTACT_TRIGGER_MS);
            maybe_landed_candidate = (ground_contact && low_alt_disarm && low_vspeed && low_hspeed && low_thrust_maybe) ? 1U : 0U;
            maybe_landed = commander_update_hysteresis(maybe_landed_candidate, &g_maybe_landed_since_ms, now_ms, LAND_MAYBE_TRIGGER_MS);
            landed = commander_update_hysteresis(maybe_landed, &g_landed_since_ms, now_ms, LAND_DETECT_TRIGGER_MS);

            if (landed)
            {
                g_land_state = SHM_LAND_STATE_LANDED;
            }
            else if (maybe_landed)
            {
                g_land_state = SHM_LAND_STATE_MAYBE_LANDED;
            }
            else if (ground_contact)
            {
                g_land_state = SHM_LAND_STATE_GROUND_CONTACT;
            }
            else
            {
                g_land_state = SHM_LAND_STATE_FLYING;
            }
        }
        else
        {
            commander_reset_land_detector();
            landed = 0U;
        }

        if ((g_nav_state == SHM_NAV_STATE_LAND) && landed)
        {
            commander_set_nav_state(SHM_NAV_STATE_NONE, now_ms);
            g_host_arm_release_required = 1U;
            g_host_control_owner = 0U;
            (void)flight_param_set_u8(FLIGHT_PARAM_ARMING_ENABLE, 0U);
            base_armed = 0U;
        }

        hard_failsafe = mandatory_ok ? 0U : 1U;
        if (hard_failsafe)
        {
            g_failsafe_reason |= SHM_FAILSAFE_REASON_MANDATORY;
            commander_set_nav_state(SHM_NAV_STATE_NONE, now_ms);
            commander_reset_land_detector();
            g_host_arm_release_required = 1U;
            g_host_control_owner = 0U;
            (void)flight_param_set_u8(FLIGHT_PARAM_ARMING_ENABLE, 0U);
        }

        out.failsafe = hard_failsafe;
        out.armed = (hard_failsafe == 0U) ? ((g_nav_state != SHM_NAV_STATE_NONE) ? 1U : (param_armed || host_armed)) : 0U;
        if (!out.armed)
        {
            commander_set_nav_state(SHM_NAV_STATE_NONE, now_ms);
            commander_reset_land_detector();
            g_host_control_owner = 0U;
        }

        out.nav_state = g_nav_state;
        out.failsafe_reason = g_failsafe_reason;
        out.land_state = g_land_state;

        if ((!out.armed) && (!hard_failsafe) && (!host.arm_request) && (param_armed == 0U))
        {
            g_failsafe_reason = SHM_FAILSAFE_REASON_NONE;
        }

        shm_publish_commander(&out);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_COMMANDER_PERIOD_MS));
    }
}

void tasks_commander_init (void)
{
    if (g_task_commander == 0)
    {
        xTaskCreate(commander_task_entry, TASK_COMMANDER_NAME, TASK_COMMANDER_STACK, 0, TASK_COMMANDER_PRIORITY, &g_task_commander);
    }
}
