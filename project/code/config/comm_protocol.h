#ifndef _COMM_PROTOCOL_H_
#define _COMM_PROTOCOL_H_

#include "zf_common_typedef.h"

#define COMM_FRAME_HEAD_0                     (0xA5U)
#define COMM_FRAME_HEAD_1                     (0x5AU)
#define COMM_MAX_PAYLOAD                      (192U)

typedef enum
{
    COMM_MSG_PING = 0x01,
    COMM_MSG_CONNECT = 0x02,
    COMM_MSG_DISCONNECT = 0x03,
    COMM_MSG_TELEM_REQ = 0x10,
    COMM_MSG_HOST_CONTROL = 0x11,
    COMM_MSG_PARAM_COUNT_REQ = 0x20,
    COMM_MSG_PARAM_GET = 0x21,
    COMM_MSG_PARAM_SET = 0x22,
    COMM_MSG_PARAM_SAVE = 0x23,
    COMM_MSG_PARAM_LOAD = 0x24,
    COMM_MSG_PARAM_RESET = 0x25,
    COMM_MSG_CALIB_START = 0x30,
    COMM_MSG_CALIB_STATUS_REQ = 0x31
} comm_msg_id_enum;

typedef enum
{
    COMM_MSG_ACK = 0x80,
    COMM_MSG_TELEM = 0x81,
    COMM_MSG_PARAM_COUNT = 0x82,
    COMM_MSG_PARAM_VALUE = 0x83,
    COMM_MSG_CALIB_STATUS = 0x84,
    COMM_MSG_CALIB_EVENT = 0x85
} comm_rsp_id_enum;

typedef enum
{
    COMM_ACK_OK = 0U,
    COMM_ACK_BAD_MSG = 1U,
    COMM_ACK_BAD_LEN = 2U,
    COMM_ACK_BAD_PARAM = 3U,
    COMM_ACK_BAD_TYPE = 4U,
    COMM_ACK_STORE_FAIL = 5U,
    COMM_ACK_BUSY = 6U,
    COMM_ACK_NOT_READY = 7U
} comm_ack_status_enum;

typedef struct
{
    uint8 msg_id;
    uint8 seq;
    uint8 payload_len;
    uint8 payload[COMM_MAX_PAYLOAD];
} comm_frame_struct;

#endif
