#include <string.h>

#include "zf_driver_flash.h"

#include "flight_params.h"

#define FLIGHT_PARAM_STORAGE_ENABLE       (0U)
#define FLIGHT_PARAM_FLASH_PAGE          (0U)
#define FLIGHT_PARAM_MAGIC               (0x46504152UL)
#define FLIGHT_PARAM_VERSION             (1U)
#define FLIGHT_PARAM_STORE_WORDS         (4U + FLIGHT_PARAM_COUNT)

typedef union
{
    uint32 u32;
    float f32;
    uint8 u8;
} flight_param_value_union;

static const uint8 g_param_types[FLIGHT_PARAM_COUNT] =
{
    FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_U8,
    FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_U8,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_U8,
    FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_U8,  FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32, FLIGHT_PARAM_TYPE_F32,
    FLIGHT_PARAM_TYPE_F32
};

static const flight_param_value_union g_param_defaults[FLIGHT_PARAM_COUNT] =
{
    [FLIGHT_PARAM_ARMING_ENABLE] = {.u8 = 0U},
    [FLIGHT_PARAM_ALT_TARGET_M] = {.f32 = 1.0f},
    [FLIGHT_PARAM_THR_BASE] = {.f32 = 350.0f},
    [FLIGHT_PARAM_THR_ALT_KP] = {.f32 = 180.0f},
    [FLIGHT_PARAM_ROLL_KP] = {.f32 = 4.0f},
    [FLIGHT_PARAM_PITCH_KP] = {.f32 = 4.0f},
    [FLIGHT_PARAM_YAW_KP] = {.f32 = 2.8f},
    [FLIGHT_PARAM_OUT_LIMIT_ROLL] = {.f32 = 220.0f},
    [FLIGHT_PARAM_OUT_LIMIT_PITCH] = {.f32 = 220.0f},
    [FLIGHT_PARAM_OUT_LIMIT_YAW] = {.f32 = 160.0f},
    [FLIGHT_PARAM_OUT_LIMIT_THR_MIN] = {.f32 = 120.0f},
    [FLIGHT_PARAM_OUT_LIMIT_THR_MAX] = {.f32 = 900.0f},
    [FLIGHT_PARAM_FLOW_ENABLE] = {.u8 = 1U},
    [FLIGHT_PARAM_TOF_ENABLE] = {.u8 = 1U},
    [FLIGHT_PARAM_RATE_ROLL_P] = {.f32 = 0.15f},
    [FLIGHT_PARAM_RATE_ROLL_I] = {.f32 = 0.20f},
    [FLIGHT_PARAM_RATE_ROLL_D] = {.f32 = 0.003f},
    [FLIGHT_PARAM_RATE_PITCH_P] = {.f32 = 0.15f},
    [FLIGHT_PARAM_RATE_PITCH_I] = {.f32 = 0.20f},
    [FLIGHT_PARAM_RATE_PITCH_D] = {.f32 = 0.003f},
    [FLIGHT_PARAM_RATE_YAW_P] = {.f32 = 0.20f},
    [FLIGHT_PARAM_RATE_YAW_I] = {.f32 = 0.10f},
    [FLIGHT_PARAM_RATE_YAW_D] = {.f32 = 0.0f},
    [FLIGHT_PARAM_RATE_INT_LIM_RP] = {.f32 = 0.30f},
    [FLIGHT_PARAM_RATE_INT_LIM_YAW] = {.f32 = 0.30f},
    [FLIGHT_PARAM_RATE_MAX_ROLL_DPS] = {.f32 = 220.0f},
    [FLIGHT_PARAM_RATE_MAX_PITCH_DPS] = {.f32 = 220.0f},
    [FLIGHT_PARAM_RATE_MAX_YAW_DPS] = {.f32 = 200.0f},
    [FLIGHT_PARAM_MOTOR_DIR_M1] = {.u8 = 0U},
    [FLIGHT_PARAM_MOTOR_DIR_M2] = {.u8 = 0U},
    [FLIGHT_PARAM_MOTOR_DIR_M3] = {.u8 = 0U},
    [FLIGHT_PARAM_MOTOR_DIR_M4] = {.u8 = 0U},
    [FLIGHT_PARAM_MOTOR_TEST_ENABLE] = {.u8 = 0U},
    [FLIGHT_PARAM_MOTOR_TEST_INDEX] = {.u8 = 0U},
    [FLIGHT_PARAM_MOTOR_TEST_THR] = {.f32 = 150.0f},
    [FLIGHT_PARAM_FLIGHT_MODE] = {.u8 = 0U},
    [FLIGHT_PARAM_FLOW_RAD_PER_COUNT] = {.f32 = 0.0012f},
    [FLIGHT_PARAM_POS_XY_P] = {.f32 = 0.8f},
    [FLIGHT_PARAM_VEL_XY_P] = {.f32 = 0.35f},
    [FLIGHT_PARAM_POS_VEL_MAX_MPS] = {.f32 = 2.0f},
    [FLIGHT_PARAM_POS_TILT_MAX_DEG] = {.f32 = 20.0f},
    [FLIGHT_PARAM_HOST_CTRL_ENABLE] = {.u8 = 0U},
    [FLIGHT_PARAM_HOST_ARM_ALLOW] = {.u8 = 0U},
    [FLIGHT_PARAM_HOST_TIMEOUT_S] = {.f32 = 0.35f},
    [FLIGHT_PARAM_HOST_XY_MAX_MPS] = {.f32 = 1.0f},
    [FLIGHT_PARAM_HOST_Z_MAX_MPS] = {.f32 = 0.8f},
    [FLIGHT_PARAM_HOST_YAW_MAX_DPS] = {.f32 = 90.0f},
    [FLIGHT_PARAM_HOST_THR_SPAN] = {.f32 = 180.0f},
    [FLIGHT_PARAM_HOST_ESTOP_ALLOW] = {.u8 = 1U},
    [FLIGHT_PARAM_LAND_HOLD_S] = {.f32 = 3.0f},
    [FLIGHT_PARAM_LAND_DESCEND_MPS] = {.f32 = 0.30f},
    [FLIGHT_PARAM_LAND_DISARM_ALT_M] = {.f32 = 0.15f},
    [FLIGHT_PARAM_THR_ALT_I] = {.f32 = 60.0f},
    [FLIGHT_PARAM_THR_ALT_I_LIM] = {.f32 = 180.0f},
    [FLIGHT_PARAM_ALT_Z_P] = {.f32 = 1.0f},
    [FLIGHT_PARAM_VEL_Z_D] = {.f32 = 30.0f},
    [FLIGHT_PARAM_VEL_Z_MAX_UP] = {.f32 = 1.5f},
    [FLIGHT_PARAM_VEL_Z_MAX_DN] = {.f32 = 0.8f},
    [FLIGHT_PARAM_MOTOR_MAP_OUT1] = {.u8 = 1U},
    [FLIGHT_PARAM_MOTOR_MAP_OUT2] = {.u8 = 2U},
    [FLIGHT_PARAM_MOTOR_MAP_OUT3] = {.u8 = 3U},
    [FLIGHT_PARAM_MOTOR_MAP_OUT4] = {.u8 = 4U},
    [FLIGHT_PARAM_HTE_ENABLE] = {.u8 = 1U},
    [FLIGHT_PARAM_HTE_HT_NOISE] = {.f32 = 6.0f},
    [FLIGHT_PARAM_HTE_HT_ERR_INIT] = {.f32 = 80.0f},
    [FLIGHT_PARAM_HTE_ACC_GATE] = {.f32 = 3.0f},
    [FLIGHT_PARAM_HTE_THR_RANGE] = {.f32 = 120.0f},
    [FLIGHT_PARAM_HTE_VXY_THR] = {.f32 = 1.5f},
    [FLIGHT_PARAM_HTE_VZ_THR] = {.f32 = 0.8f}
};

static flight_param_value_union g_param_values[FLIGHT_PARAM_COUNT];
static uint8 g_inited = 0U;

static uint32 fp_crc32_step (uint32 crc, uint32 data)
{
    uint8 i = 0U;
    crc ^= data;
    for (i = 0U; i < 32U; i++)
    {
        crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
    }
    return crc;
}

static uint32 fp_compute_crc_words (const uint32 *data, uint32 words)
{
    uint32 crc = 0xFFFFFFFFUL;
    uint32 i = 0U;
    for (i = 0U; i < words; i++)
    {
        crc = fp_crc32_step(crc, data[i]);
    }
    return ~crc;
}

static uint8 fp_param_valid (uint8 param_id)
{
    return (param_id < FLIGHT_PARAM_COUNT) ? 1U : 0U;
}

static uint8 fp_bool_u8 (uint8 value)
{
    return (value != 0U) ? 1U : 0U;
}

static void fp_sanitize_u8 (uint8 param_id, flight_param_value_union *value)
{
    if (value == 0)
    {
        return;
    }

    switch (param_id)
    {
        case FLIGHT_PARAM_ARMING_ENABLE:
        case FLIGHT_PARAM_FLOW_ENABLE:
        case FLIGHT_PARAM_TOF_ENABLE:
        case FLIGHT_PARAM_MOTOR_DIR_M1:
        case FLIGHT_PARAM_MOTOR_DIR_M2:
        case FLIGHT_PARAM_MOTOR_DIR_M3:
        case FLIGHT_PARAM_MOTOR_DIR_M4:
        case FLIGHT_PARAM_MOTOR_TEST_ENABLE:
        case FLIGHT_PARAM_HOST_CTRL_ENABLE:
        case FLIGHT_PARAM_HOST_ARM_ALLOW:
        case FLIGHT_PARAM_HOST_ESTOP_ALLOW:
        case FLIGHT_PARAM_HTE_ENABLE:
            value->u8 = fp_bool_u8(value->u8);
            break;

        case FLIGHT_PARAM_MOTOR_TEST_INDEX:
            if (value->u8 > 5U)
            {
                value->u8 = 0U;
            }
            break;

        case FLIGHT_PARAM_FLIGHT_MODE:
            if (value->u8 > 1U)
            {
                value->u8 = 0U;
            }
            break;

        case FLIGHT_PARAM_MOTOR_MAP_OUT1:
        case FLIGHT_PARAM_MOTOR_MAP_OUT2:
        case FLIGHT_PARAM_MOTOR_MAP_OUT3:
        case FLIGHT_PARAM_MOTOR_MAP_OUT4:
            if ((value->u8 < 1U) || (value->u8 > 4U))
            {
                value->u8 = (uint8)(param_id - FLIGHT_PARAM_MOTOR_MAP_OUT1 + 1U);
            }
            break;

        default:
            break;
    }
}

static void fp_reset_defaults_locked (void)
{
    memcpy(g_param_values, g_param_defaults, sizeof(g_param_values));
}

void flight_params_reset_defaults (void)
{
    fp_reset_defaults_locked();
}

uint8 flight_param_get_type (uint8 param_id)
{
    if (!fp_param_valid(param_id))
    {
        return 0U;
    }
    return g_param_types[param_id];
}

uint8 flight_param_get_u8 (uint8 param_id, uint8 *value)
{
    if ((!fp_param_valid(param_id)) || (value == 0) || (g_param_types[param_id] != FLIGHT_PARAM_TYPE_U8))
    {
        return 1U;
    }
    *value = g_param_values[param_id].u8;
    return 0U;
}

uint8 flight_param_get_f32 (uint8 param_id, float *value)
{
    if ((!fp_param_valid(param_id)) || (value == 0) || (g_param_types[param_id] != FLIGHT_PARAM_TYPE_F32))
    {
        return 1U;
    }
    *value = g_param_values[param_id].f32;
    return 0U;
}

uint8 flight_param_set_u8 (uint8 param_id, uint8 value)
{
    if ((!fp_param_valid(param_id)) || (g_param_types[param_id] != FLIGHT_PARAM_TYPE_U8))
    {
        return 1U;
    }
    g_param_values[param_id].u8 = value;
    fp_sanitize_u8(param_id, &g_param_values[param_id]);
    return 0U;
}

uint8 flight_param_set_f32 (uint8 param_id, float value)
{
    if ((!fp_param_valid(param_id)) || (g_param_types[param_id] != FLIGHT_PARAM_TYPE_F32))
    {
        return 1U;
    }
    g_param_values[param_id].f32 = value;
    return 0U;
}

uint8 flight_param_get_u8_default (uint8 param_id, uint8 default_value)
{
    uint8 value = default_value;
    if (flight_param_get_u8(param_id, &value) != 0U)
    {
        return default_value;
    }
    return value;
}

float flight_param_get_f32_default (uint8 param_id, float default_value)
{
    float value = default_value;
    if (flight_param_get_f32(param_id, &value) != 0U)
    {
        return default_value;
    }
    return value;
}

uint16 flight_param_count (void)
{
    return FLIGHT_PARAM_COUNT;
}

uint8 flight_params_load (void)
{
#if !FLIGHT_PARAM_STORAGE_ENABLE
    return 0U;
#else
    uint32 stored_crc = 0U;
    uint32 calc_crc = 0U;
    uint16 i = 0U;

    if (!g_inited)
    {
        flash_init();
    }

    flash_read_page_to_buffer(0U, FLIGHT_PARAM_FLASH_PAGE, FLIGHT_PARAM_STORE_WORDS);

    if (flash_union_buffer[0].uint32_type != FLIGHT_PARAM_MAGIC)
    {
        return 1U;
    }
    if (flash_union_buffer[1].uint32_type != FLIGHT_PARAM_VERSION)
    {
        return 2U;
    }
    if (flash_union_buffer[2].uint32_type != FLIGHT_PARAM_COUNT)
    {
        return 3U;
    }

    stored_crc = flash_union_buffer[3U + FLIGHT_PARAM_COUNT].uint32_type;
    calc_crc = fp_compute_crc_words((const uint32 *)flash_union_buffer, 3U + FLIGHT_PARAM_COUNT);
    if (stored_crc != calc_crc)
    {
        return 4U;
    }

    for (i = 0U; i < FLIGHT_PARAM_COUNT; i++)
    {
        g_param_values[i].u32 = flash_union_buffer[3U + i].uint32_type;
        if (g_param_types[i] == FLIGHT_PARAM_TYPE_U8)
        {
            fp_sanitize_u8((uint8)i, &g_param_values[i]);
        }
    }

    return 0U;
#endif
}

uint8 flight_params_save (void)
{
#if !FLIGHT_PARAM_STORAGE_ENABLE
    return 0U;
#else
    uint32 crc = 0U;
    uint16 i = 0U;

    if (!g_inited)
    {
        flight_params_init();
    }

    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = FLIGHT_PARAM_MAGIC;
    flash_union_buffer[1].uint32_type = FLIGHT_PARAM_VERSION;
    flash_union_buffer[2].uint32_type = FLIGHT_PARAM_COUNT;

    for (i = 0U; i < FLIGHT_PARAM_COUNT; i++)
    {
        flash_union_buffer[3U + i].uint32_type = g_param_values[i].u32;
    }

    crc = fp_compute_crc_words((const uint32 *)flash_union_buffer, 3U + FLIGHT_PARAM_COUNT);
    flash_union_buffer[3U + FLIGHT_PARAM_COUNT].uint32_type = crc;
    return flash_write_page_from_buffer(0U, FLIGHT_PARAM_FLASH_PAGE, FLIGHT_PARAM_STORE_WORDS);
#endif
}

void flight_params_init (void)
{
    if (g_inited)
    {
        return;
    }

    fp_reset_defaults_locked();
    if (flight_params_load() != 0U)
    {
        fp_reset_defaults_locked();
    }
    g_inited = 1U;
}
