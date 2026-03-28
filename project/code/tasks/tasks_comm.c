#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config/comm_protocol.h"
#include "config/flight_params.h"
#include "config/shared_memory.h"
#include "tasks.h"
#include "zf_device_wireless_uart.h"

#define TASK_COMM_NAME                 "Comm"
#define TASK_COMM_PERIOD_MS            (10U)
#define TASK_COMM_STACK                (768U)
#define TASK_COMM_PRIORITY             (3U)

#define COMM_PARAM_AUTOSAVE_DELAY_MS   (1200U)
#define COMM_SESSION_TIMEOUT_MS        (3000U)
#define COMM_HOST_CTRL_PAYLOAD_LEN     (10U)
#define COMM_AXIS_SCALE                (1000.0f)

typedef enum
{
    RX_WAIT_H0 = 0,
    RX_WAIT_H1 = 1,
    RX_WAIT_MSG = 2,
    RX_WAIT_SEQ = 3,
    RX_WAIT_LEN = 4,
    RX_WAIT_PAYLOAD = 5,
    RX_WAIT_CRC0 = 6,
    RX_WAIT_CRC1 = 7
} comm_rx_state_enum;

typedef struct
{
    uint8 imu_calibrated;
    uint8 gyro_calibrated;
    uint8 mag_calibrated;
    uint8 imu_running;
    uint8 gyro_running;
    uint8 mag_running;
    uint8 imu_progress;
    uint8 gyro_progress;
    uint8 mag_progress;
    uint8 imu_acc_mask;
    uint8 imu_gyro_mask;
    uint8 last_result;
} comm_calibration_status_struct;

static TaskHandle_t g_task_comm = 0;
static uint8 g_uart_ready = 0U;
static uint32 g_tx_count = 0U;
static uint32 g_rx_count = 0U;
static uint32 g_last_rx_ms = 0U;
static uint8 g_session_connected = 0U;
static uint8 g_param_dirty = 0U;
static uint32 g_param_save_deadline_ms = 0U;
static comm_frame_struct g_rx_frame = {0};
static comm_rx_state_enum g_rx_state = RX_WAIT_H0;
static uint8 g_rx_index = 0U;
static uint16 g_rx_crc = 0U;
static uint16 g_rx_crc_calc = 0U;

static float comm_decode_axis_i16 (const uint8 *src)
{
    int16 raw = 0;
    float value = 0.0f;

    if (src == 0)
    {
        return 0.0f;
    }

    memcpy(&raw, src, sizeof(raw));
    value = (float)raw / COMM_AXIS_SCALE;
    if (value > 1.0f)
    {
        value = 1.0f;
    }
    else if (value < -1.0f)
    {
        value = -1.0f;
    }
    return value;
}

static uint32 task_now_ms (void)
{
    return (uint32)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint16 comm_crc16_update (uint16 crc, uint8 data)
{
    uint8 i = 0U;

    crc ^= data;
    for (i = 0U; i < 8U; i++)
    {
        if (crc & 1U)
        {
            crc = (uint16)((crc >> 1U) ^ 0xA001U);
        }
        else
        {
            crc >>= 1U;
        }
    }

    return crc;
}

static void comm_publish_host_control (uint8 link_active,
                                       uint8 arm_request,
                                       uint8 control_active,
                                       uint8 land_request,
                                       uint8 estop_request,
                                       float forward,
                                       float right,
                                       float up,
                                       float yaw)
{
    shm_host_control_data_struct host = {0};

    host.timestamp_ms = task_now_ms();
    host.link_active = link_active ? 1U : 0U;
    host.arm_request = arm_request ? 1U : 0U;
    host.control_active = control_active ? 1U : 0U;
    host.land_request = land_request ? 1U : 0U;
    host.estop_request = estop_request ? 1U : 0U;
    host.forward = forward;
    host.right = right;
    host.up = up;
    host.yaw = yaw;
    shm_publish_host_control(&host);
}

static void comm_clear_host_control (void)
{
    comm_publish_host_control(0U, 0U, 0U, 0U, 0U, 0.0f, 0.0f, 0.0f, 0.0f);
}

static void comm_send_frame (uint8 msg_id, uint8 seq, const uint8 *payload, uint8 payload_len)
{
    uint8 frame[2 + 1 + 1 + 1 + COMM_MAX_PAYLOAD + 2] = {0};
    uint16 crc = 0U;
    uint16 idx = 0U;
    uint8 i = 0U;

    if (payload_len > COMM_MAX_PAYLOAD)
    {
        return;
    }

    frame[idx++] = COMM_FRAME_HEAD_0;
    frame[idx++] = COMM_FRAME_HEAD_1;
    frame[idx++] = msg_id;
    frame[idx++] = seq;
    frame[idx++] = payload_len;
    if ((payload_len > 0U) && (payload != 0))
    {
        memcpy(&frame[idx], payload, payload_len);
        idx += payload_len;
    }

    crc = 0xFFFFU;
    crc = comm_crc16_update(crc, msg_id);
    crc = comm_crc16_update(crc, seq);
    crc = comm_crc16_update(crc, payload_len);
    for (i = 0U; i < payload_len; i++)
    {
        crc = comm_crc16_update(crc, payload[i]);
    }

    frame[idx++] = (uint8)(crc & 0xFFU);
    frame[idx++] = (uint8)((crc >> 8U) & 0xFFU);

    if (g_uart_ready)
    {
        (void)wireless_uart_send_buffer(frame, idx);
        g_tx_count++;
    }
}

static void comm_send_ack (uint8 seq, uint8 req_msg, uint8 status)
{
    uint8 payload[2];

    payload[0] = req_msg;
    payload[1] = status;
    comm_send_frame(COMM_MSG_ACK, seq, payload, sizeof(payload));
}

static void comm_send_param_count (uint8 seq)
{
    uint8 payload[2];
    uint16 count = flight_param_count();

    payload[0] = (uint8)(count & 0xFFU);
    payload[1] = (uint8)((count >> 8U) & 0xFFU);
    comm_send_frame(COMM_MSG_PARAM_COUNT, seq, payload, sizeof(payload));
}

static void comm_send_param_value (uint8 seq, uint8 param_id)
{
    uint8 payload[6] = {0};
    uint8 type = flight_param_get_type(param_id);
    float f32 = 0.0f;
    uint8 u8 = 0U;

    payload[0] = param_id;
    payload[1] = type;

    if (type == FLIGHT_PARAM_TYPE_F32)
    {
        if (flight_param_get_f32(param_id, &f32) != 0U)
        {
            return;
        }
        memcpy(&payload[2], &f32, sizeof(f32));
    }
    else if (type == FLIGHT_PARAM_TYPE_U8)
    {
        if (flight_param_get_u8(param_id, &u8) != 0U)
        {
            return;
        }
        payload[2] = u8;
    }
    else
    {
        return;
    }

    comm_send_frame(COMM_MSG_PARAM_VALUE, seq, payload, sizeof(payload));
}

static void comm_send_all_params (uint8 seq)
{
    uint16 count = flight_param_count();
    uint16 i = 0U;

    comm_send_param_count(seq);
    for (i = 0U; i < count; i++)
    {
        comm_send_param_value(seq, (uint8)i);
    }
}

static void comm_send_telem (uint8 seq)
{
    uint8 payload[COMM_MAX_PAYLOAD] = {0};
    shm_estimator_data_struct est = {0};
    shm_control_data_struct ctl = {0};
    shm_commander_data_struct cmd = {0};
    shm_mixer_data_struct mix = {0};
    shm_imu_data_struct imu0 = {0};
    shm_imu_data_struct imu1 = {0};
    shm_mag_data_struct mag = {0};
    shm_baro_data_struct baro = {0};
    shm_tof_data_struct tof = {0};
    shm_flow_data_struct flow = {0};
    comm_calibration_status_struct cal = {0};
    uint8 imu_healthy = 0U;
    uint16 idx = 0U;

    (void)shm_read_estimator(&est, 0);
    (void)shm_read_control(&ctl, 0);
    (void)shm_read_commander(&cmd, 0);
    (void)shm_read_mixer(&mix, 0);
    (void)shm_read_imu(0U, &imu0, 0);
    (void)shm_read_imu(1U, &imu1, 0);
    (void)shm_read_mag(&mag, 0);
    (void)shm_read_baro(&baro, 0);
    (void)shm_read_tof(&tof, 0);
    (void)shm_read_flow(&flow, 0);
    imu_healthy = (imu0.healthy || imu1.healthy) ? 1U : 0U;

    memcpy(&payload[idx], &est.roll_rad, 4U); idx += 4U;
    memcpy(&payload[idx], &est.pitch_rad, 4U); idx += 4U;
    memcpy(&payload[idx], &est.yaw_rad, 4U); idx += 4U;
    memcpy(&payload[idx], &est.altitude_m, 4U); idx += 4U;
    payload[idx++] = cmd.armed;
    payload[idx++] = cmd.failsafe;
    payload[idx++] = cmd.flight_mode;
    payload[idx++] = cmd.nav_state;
    payload[idx++] = cmd.failsafe_reason;
    payload[idx++] = cmd.land_state;
    payload[idx++] = est.healthy;
    payload[idx++] = est.flow_fused;
    payload[idx++] = est.active_imu;
    payload[idx++] = est.altitude_source;
    payload[idx++] = est.accel_fused;
    payload[idx++] = est.alt_rejected;
    payload[idx++] = est.flow_rejected;
    payload[idx++] = ctl.armed;
    payload[idx++] = ctl.failsafe;
    payload[idx++] = mix.enabled;
    payload[idx++] = mix.saturation_high;
    payload[idx++] = mix.saturation_low;
    payload[idx++] = mix.yaw_limited;
    payload[idx++] = mix.thrust_reduced;
    payload[idx++] = tof.healthy;
    payload[idx++] = tof.data_ready;
    memcpy(&payload[idx], &tof.distance_mm, 2U); idx += 2U;
    memcpy(&payload[idx], &flow.delta_x, 2U); idx += 2U;
    memcpy(&payload[idx], &flow.delta_y, 2U); idx += 2U;
    payload[idx++] = flow.squal;
    payload[idx++] = flow.healthy;
    payload[idx++] = imu_healthy;
    payload[idx++] = mag.healthy;
    payload[idx++] = baro.healthy;
    payload[idx++] = cal.imu_calibrated;
    payload[idx++] = cal.mag_calibrated;
    memcpy(&payload[idx], &est.pos_x_m, 4U); idx += 4U;
    memcpy(&payload[idx], &est.pos_y_m, 4U); idx += 4U;
    memcpy(&payload[idx], &est.vel_x_mps, 4U); idx += 4U;
    memcpy(&payload[idx], &est.vel_y_mps, 4U); idx += 4U;
    memcpy(&payload[idx], &mix.yaw_scale, 4U); idx += 4U;
    memcpy(&payload[idx], &mix.collective_offset, 4U); idx += 4U;
    payload[idx++] = ctl.telemetry_flags;
    memcpy(&payload[idx], &ctl.altitude_sp_m, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.vel_sp_z_mps, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.alt_i_term, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.rate_sp_roll_rps, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.rate_sp_pitch_rps, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.rate_sp_yaw_rps, 4U); idx += 4U;
    memcpy(&payload[idx], &est.vertical_speed_mps, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.vel_sp_x_mps, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.vel_sp_y_mps, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.roll_sp_rad, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.pitch_sp_rad, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.rate_meas_roll_rps, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.rate_meas_pitch_rps, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.rate_meas_yaw_rps, 4U); idx += 4U;
    payload[idx++] = ctl.hover_thrust_status;
    payload[idx++] = ctl.hover_thrust_valid;
    memcpy(&payload[idx], &ctl.hover_thrust_est, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.hover_thrust_var, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.hover_thrust_innov, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.hover_thrust_innov_var, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.hover_thrust_test_ratio, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.hover_thrust_accel_mps2, 4U); idx += 4U;
    memcpy(&payload[idx], &ctl.hover_thrust_accel_noise, 4U); idx += 4U;

    comm_send_frame(COMM_MSG_TELEM, seq, payload, (uint8)idx);
}

static void comm_send_calib_status (uint8 seq)
{
    uint8 payload[12] = {0};
    comm_send_frame(COMM_MSG_CALIB_STATUS, seq, payload, sizeof(payload));
}

static void comm_handle_frame (const comm_frame_struct *frm)
{
    uint8 param_id = 0U;
    uint8 type = 0U;
    float f32_val = 0.0f;
    uint8 u8_val = 0U;
    uint8 status = COMM_ACK_OK;

    if ((frm->msg_id != COMM_MSG_PING) &&
        (frm->msg_id != COMM_MSG_CONNECT) &&
        (!g_session_connected))
    {
        comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_BAD_MSG);
        return;
    }

    switch (frm->msg_id)
    {
        case COMM_MSG_PING:
            comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_OK);
            break;

        case COMM_MSG_CONNECT:
            g_session_connected = 1U;
            g_last_rx_ms = task_now_ms();
            comm_clear_host_control();
            comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_OK);
            comm_send_all_params(frm->seq);
            break;

        case COMM_MSG_DISCONNECT:
            g_session_connected = 0U;
            comm_clear_host_control();
            comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_OK);
            break;

        case COMM_MSG_TELEM_REQ:
            comm_send_telem(frm->seq);
            break;

        case COMM_MSG_HOST_CONTROL:
            if (frm->payload_len != COMM_HOST_CTRL_PAYLOAD_LEN)
            {
                comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_BAD_LEN);
                break;
            }

            comm_publish_host_control(1U,
                                      frm->payload[0],
                                      (uint8)(frm->payload[1] & 0x01U),
                                      (uint8)((frm->payload[1] >> 1U) & 0x01U),
                                      (uint8)((frm->payload[1] >> 2U) & 0x01U),
                                      comm_decode_axis_i16(&frm->payload[2]),
                                      comm_decode_axis_i16(&frm->payload[4]),
                                      comm_decode_axis_i16(&frm->payload[6]),
                                      comm_decode_axis_i16(&frm->payload[8]));
            comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_OK);
            break;

        case COMM_MSG_PARAM_COUNT_REQ:
            comm_send_param_count(frm->seq);
            break;

        case COMM_MSG_PARAM_GET:
            if (frm->payload_len != 1U)
            {
                comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_BAD_LEN);
                break;
            }
            param_id = frm->payload[0];
            if (flight_param_get_type(param_id) == 0U)
            {
                comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_BAD_PARAM);
                break;
            }
            comm_send_param_value(frm->seq, param_id);
            break;

        case COMM_MSG_PARAM_SET:
            if (frm->payload_len != 6U)
            {
                comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_BAD_LEN);
                break;
            }
            param_id = frm->payload[0];
            type = frm->payload[1];
            if (flight_param_get_type(param_id) == 0U)
            {
                comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_BAD_PARAM);
                break;
            }

            if (type == FLIGHT_PARAM_TYPE_F32)
            {
                memcpy(&f32_val, &frm->payload[2], 4U);
                status = (flight_param_set_f32(param_id, f32_val) == 0U) ? COMM_ACK_OK : COMM_ACK_BAD_PARAM;
            }
            else if (type == FLIGHT_PARAM_TYPE_U8)
            {
                u8_val = frm->payload[2];
                status = (flight_param_set_u8(param_id, u8_val) == 0U) ? COMM_ACK_OK : COMM_ACK_BAD_PARAM;
            }
            else
            {
                status = COMM_ACK_BAD_TYPE;
            }

            comm_send_ack(frm->seq, frm->msg_id, status);
            if (status == COMM_ACK_OK)
            {
                comm_send_param_value(frm->seq, param_id);
                g_param_dirty = 1U;
                g_param_save_deadline_ms = task_now_ms() + COMM_PARAM_AUTOSAVE_DELAY_MS;
            }
            break;

        case COMM_MSG_PARAM_SAVE:
            status = (flight_params_save() == 0U) ? COMM_ACK_OK : COMM_ACK_STORE_FAIL;
            comm_send_ack(frm->seq, frm->msg_id, status);
            break;

        case COMM_MSG_PARAM_LOAD:
            status = (flight_params_load() == 0U) ? COMM_ACK_OK : COMM_ACK_STORE_FAIL;
            comm_send_ack(frm->seq, frm->msg_id, status);
            break;

        case COMM_MSG_PARAM_RESET:
            flight_params_reset_defaults();
            comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_OK);
            break;

        case COMM_MSG_CALIB_START:
            comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_NOT_READY);
            comm_send_calib_status(frm->seq);
            break;

        case COMM_MSG_CALIB_STATUS_REQ:
            comm_send_calib_status(frm->seq);
            break;

        default:
            comm_send_ack(frm->seq, frm->msg_id, COMM_ACK_BAD_MSG);
            break;
    }
}

static void comm_parser_consume_byte (uint8 ch)
{
    switch (g_rx_state)
    {
        case RX_WAIT_H0:
            if (ch == COMM_FRAME_HEAD_0)
            {
                g_rx_state = RX_WAIT_H1;
            }
            break;

        case RX_WAIT_H1:
            if (ch == COMM_FRAME_HEAD_1)
            {
                g_rx_state = RX_WAIT_MSG;
                g_rx_crc_calc = 0xFFFFU;
                g_rx_index = 0U;
            }
            else
            {
                g_rx_state = RX_WAIT_H0;
            }
            break;

        case RX_WAIT_MSG:
            g_rx_frame.msg_id = ch;
            g_rx_crc_calc = comm_crc16_update(g_rx_crc_calc, ch);
            g_rx_state = RX_WAIT_SEQ;
            break;

        case RX_WAIT_SEQ:
            g_rx_frame.seq = ch;
            g_rx_crc_calc = comm_crc16_update(g_rx_crc_calc, ch);
            g_rx_state = RX_WAIT_LEN;
            break;

        case RX_WAIT_LEN:
            g_rx_frame.payload_len = ch;
            g_rx_crc_calc = comm_crc16_update(g_rx_crc_calc, ch);
            if (g_rx_frame.payload_len > COMM_MAX_PAYLOAD)
            {
                g_rx_state = RX_WAIT_H0;
            }
            else if (g_rx_frame.payload_len == 0U)
            {
                g_rx_state = RX_WAIT_CRC0;
            }
            else
            {
                g_rx_index = 0U;
                g_rx_state = RX_WAIT_PAYLOAD;
            }
            break;

        case RX_WAIT_PAYLOAD:
            g_rx_frame.payload[g_rx_index++] = ch;
            g_rx_crc_calc = comm_crc16_update(g_rx_crc_calc, ch);
            if (g_rx_index >= g_rx_frame.payload_len)
            {
                g_rx_state = RX_WAIT_CRC0;
            }
            break;

        case RX_WAIT_CRC0:
            g_rx_crc = ch;
            g_rx_state = RX_WAIT_CRC1;
            break;

        case RX_WAIT_CRC1:
            g_rx_crc |= (uint16)((uint16)ch << 8U);
            if (g_rx_crc == g_rx_crc_calc)
            {
                g_rx_count++;
                g_last_rx_ms = task_now_ms();
                comm_handle_frame(&g_rx_frame);
            }
            g_rx_state = RX_WAIT_H0;
            break;

        default:
            g_rx_state = RX_WAIT_H0;
            break;
    }
}

static void comm_task_entry (void *parameter)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32 last_telem_ms = 0U;

    (void)parameter;

    while (1)
    {
        uint8 rx_buf[32];
        uint32 got = wireless_uart_read_buffer(rx_buf, sizeof(rx_buf));
        shm_comm_data_struct out = {0};
        uint32 now_ms = task_now_ms();
        uint32 i = 0U;

        if (got > 0U)
        {
            for (i = 0U; i < got; i++)
            {
                comm_parser_consume_byte(rx_buf[i]);
            }
        }

        now_ms = task_now_ms();
        if (g_session_connected && ((now_ms - g_last_rx_ms) >= COMM_SESSION_TIMEOUT_MS))
        {
            g_session_connected = 0U;
            comm_clear_host_control();
        }

        if (g_session_connected && ((now_ms - last_telem_ms) >= 100U))
        {
            comm_send_telem(0xFFU);
            last_telem_ms = now_ms;
        }

        if (g_param_dirty && ((int32)(now_ms - g_param_save_deadline_ms) >= 0))
        {
            if (flight_params_save() == 0U)
            {
                g_param_dirty = 0U;
            }
            else
            {
                g_param_save_deadline_ms = now_ms + COMM_PARAM_AUTOSAVE_DELAY_MS;
            }
        }

        out.timestamp_ms = now_ms;
        out.link_up = g_session_connected ? 1U : 0U;
        out.tx_count = g_tx_count;
        out.rx_count = g_rx_count;
        shm_publish_comm(&out);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_COMM_PERIOD_MS));
    }
}

void tasks_comm_init (void)
{
    if (g_task_comm == 0)
    {
        g_uart_ready = (wireless_uart_init() == 0U) ? 1U : 0U;
        g_last_rx_ms = task_now_ms();
        comm_clear_host_control();
        xTaskCreate(comm_task_entry, TASK_COMM_NAME, TASK_COMM_STACK, 0, TASK_COMM_PRIORITY, &g_task_comm);
    }
}
