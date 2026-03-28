#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config/flight_params.h"
#include "config/shared_memory.h"
#include "drivers/driver_pmw3901.h"
#include "drivers/driver_sch16tk01.h"
#include "drivers/driver_vl53l1x.h"
#include "tasks.h"
#include "zf_driver_delay.h"

#define SENSOR_IMU_TASK_NAME                 "SensorIMU"
#define SENSOR_FLOW_TASK_NAME                "SensorFLOW"
#define SENSOR_TOF_TASK_NAME                 "SensorTOF"

#define SENSOR_IMU_TASK_STACK_WORDS          (768U)
#define SENSOR_FLOW_TASK_STACK_WORDS         (384U)
#define SENSOR_TOF_TASK_STACK_WORDS          (384U)

#define SENSOR_IMU_TASK_PRIORITY             (4U)
#define SENSOR_FLOW_TASK_PRIORITY            (2U)
#define SENSOR_TOF_TASK_PRIORITY             (2U)

#define SENSOR_IMU_PERIOD_MS                 (5U)
#define SENSOR_FLOW_PERIOD_MS                (10U)
#define SENSOR_TOF_PERIOD_MS                 (50U)
#define SENSOR_OPTIONAL_RETRY_MS             (1000U)

#define SCH16TK01_ACC_SCALE_MSS_PER_LSB      (1.0f / 3200.0f)
#define SCH16TK01_ACC_SCALE_G_PER_LSB        (SCH16TK01_ACC_SCALE_MSS_PER_LSB / 9.80665f)
#define SCH16TK01_GYRO_SCALE_REV1_DPS_PER_LSB (1.0f / 1600.0f)
#define SCH16TK01_GYRO_SCALE_REV2_DPS_PER_LSB (1.0f / 100.0f)
#define SCH16TK01_GYRO_BIAS_CALIB_SAMPLES    (200U)
#define SCH16TK01_GYRO_BIAS_CALIB_DELAY_MS   (5U)

static TaskHandle_t g_imu_task_handle = 0;
static TaskHandle_t g_flow_task_handle = 0;
static TaskHandle_t g_tof_task_handle = 0;

static float g_gyro_bias_x_dps = 0.0f;
static float g_gyro_bias_y_dps = 0.0f;
static float g_gyro_bias_z_dps = 0.0f;

static uint32 sensor_tick_to_ms (TickType_t tick)
{
    return (uint32)(tick * portTICK_PERIOD_MS);
}

static int16 sensor_clamp_i16 (int32 value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16)value;
}

static float sch16tk01_get_gyro_scale_dps_per_lsb (uint8 chip_version)
{
    if (chip_version == DRIVER_SCH16TK01_CHIP_VERSION_REV2)
    {
        return SCH16TK01_GYRO_SCALE_REV2_DPS_PER_LSB;
    }

    return SCH16TK01_GYRO_SCALE_REV1_DPS_PER_LSB;
}

static void sensor_fill_flow_diag (shm_flow_data_struct *shm_data)
{
    driver_pmw3901_status_struct status = driver_pmw3901_get_status();

    if (shm_data == 0)
    {
        return;
    }

    shm_data->detect_status = status.detect_status;
    shm_data->expected_chip_id = status.expected_chip_id;
    shm_data->detected_chip_id = status.chip_id;
    shm_data->detected_chip_id_inverse = status.chip_id_inverse;
}

static void sensor_fill_tof_diag (shm_tof_data_struct *shm_data)
{
    driver_vl53l1x_status_struct status = driver_vl53l1x_get_status();

    if (shm_data == 0)
    {
        return;
    }

    shm_data->detect_status = status.detect_status;
    shm_data->driver_type = SHM_TOF_DRIVER_VL53L1X;
    shm_data->detected_type = status.detected_type;
    shm_data->expected_model_id = status.expected_model_id;
    shm_data->detected_model_id = status.model_id;
    shm_data->detected_alt_model_id = 0U;
}

static void sensor_publish_imu_not_ready (TickType_t now_tick, uint8 error_code)
{
    shm_imu_data_struct shm_data;

    memset(&shm_data, 0, sizeof(shm_data));
    shm_data.timestamp_ms = sensor_tick_to_ms(now_tick);
    shm_data.healthy = 0U;
    shm_data.error_code = error_code;
    shm_publish_imu(0U, &shm_data);
}

static void sensor_publish_optional_disabled_flow (TickType_t now_tick)
{
    shm_flow_data_struct shm_data;

    memset(&shm_data, 0, sizeof(shm_data));
    shm_data.timestamp_ms = sensor_tick_to_ms(now_tick);
    shm_data.healthy = 0U;
    shm_data.error_code = 0xF1U;
    sensor_fill_flow_diag(&shm_data);
    shm_publish_flow(&shm_data);
}

static void sensor_publish_optional_disabled_tof (TickType_t now_tick)
{
    shm_tof_data_struct shm_data;

    memset(&shm_data, 0, sizeof(shm_data));
    shm_data.timestamp_ms = sensor_tick_to_ms(now_tick);
    shm_data.healthy = 0U;
    shm_data.error_code = 0xF1U;
    sensor_fill_tof_diag(&shm_data);
    shm_publish_tof(&shm_data);
}

static void sch16tk01_calibrate_gyro_bias (void)
{
    driver_sch16tk01_data_struct sample = {0};
    driver_sch16tk01_status_struct status = driver_sch16tk01_get_status();
    float gyro_scale_dps_per_lsb = sch16tk01_get_gyro_scale_dps_per_lsb(status.chip_version);
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint16 valid_samples = 0U;
    uint16 i = 0U;

    g_gyro_bias_x_dps = 0.0f;
    g_gyro_bias_y_dps = 0.0f;
    g_gyro_bias_z_dps = 0.0f;

    for (i = 0U; i < SCH16TK01_GYRO_BIAS_CALIB_SAMPLES; i++)
    {
        if (0U == driver_sch16tk01_read(&sample))
        {
            sum_x += (float)sample.gyro_x_raw * gyro_scale_dps_per_lsb;
            sum_y += (float)sample.gyro_y_raw * gyro_scale_dps_per_lsb;
            sum_z += (float)sample.gyro_z_raw * gyro_scale_dps_per_lsb;
            valid_samples++;
        }
        system_delay_ms(SCH16TK01_GYRO_BIAS_CALIB_DELAY_MS);
    }

    if (valid_samples > 0U)
    {
        g_gyro_bias_x_dps = sum_x / (float)valid_samples;
        g_gyro_bias_y_dps = sum_y / (float)valid_samples;
        g_gyro_bias_z_dps = sum_z / (float)valid_samples;
    }
}

static uint8 sensor_publish_imu (TickType_t now_tick)
{
    driver_sch16tk01_data_struct raw = {0};
    driver_sch16tk01_status_struct status = driver_sch16tk01_get_status();
    shm_imu_data_struct shm_data;
    float gyro_scale_dps_per_lsb = sch16tk01_get_gyro_scale_dps_per_lsb(status.chip_version);
    uint8 ret = 0U;

    memset(&shm_data, 0, sizeof(shm_data));
    ret = driver_sch16tk01_read(&raw);

    shm_data.timestamp_ms = sensor_tick_to_ms(now_tick);
    shm_data.healthy = (ret == 0U) ? 1U : 0U;
    shm_data.error_code = ret;

    if (ret == 0U)
    {
        shm_data.accel_x_raw = sensor_clamp_i16(raw.acc_x_raw);
        shm_data.accel_y_raw = sensor_clamp_i16(raw.acc_y_raw);
        shm_data.accel_z_raw = sensor_clamp_i16(raw.acc_z_raw);
        shm_data.gyro_x_raw = sensor_clamp_i16(raw.gyro_x_raw);
        shm_data.gyro_y_raw = sensor_clamp_i16(raw.gyro_y_raw);
        shm_data.gyro_z_raw = sensor_clamp_i16(raw.gyro_z_raw);

        shm_data.accel_x_g = (float)raw.acc_x_raw * SCH16TK01_ACC_SCALE_G_PER_LSB;
        shm_data.accel_y_g = (float)raw.acc_y_raw * SCH16TK01_ACC_SCALE_G_PER_LSB;
        shm_data.accel_z_g = (float)raw.acc_z_raw * SCH16TK01_ACC_SCALE_G_PER_LSB;
        shm_data.gyro_x_dps = (float)raw.gyro_x_raw * gyro_scale_dps_per_lsb - g_gyro_bias_x_dps;
        shm_data.gyro_y_dps = (float)raw.gyro_y_raw * gyro_scale_dps_per_lsb - g_gyro_bias_y_dps;
        shm_data.gyro_z_dps = (float)raw.gyro_z_raw * gyro_scale_dps_per_lsb - g_gyro_bias_z_dps;
    }

    shm_publish_imu(0U, &shm_data);
    return ret;
}

static void sensor_publish_flow (TickType_t now_tick)
{
    driver_pmw3901_motion_struct motion = {0};
    shm_flow_data_struct shm_data;
    uint8 ret = 0U;

    memset(&shm_data, 0, sizeof(shm_data));
    ret = driver_pmw3901_read_motion(&motion);

    shm_data.timestamp_ms = sensor_tick_to_ms(now_tick);
    shm_data.healthy = (ret == 0U) ? 1U : 0U;
    shm_data.error_code = ret;
    sensor_fill_flow_diag(&shm_data);

    if (ret == 0U)
    {
        shm_data.delta_x = motion.delta_x;
        shm_data.delta_y = motion.delta_y;
        shm_data.motion = motion.motion;
        shm_data.squal = motion.squal;
        shm_data.raw_sum = motion.raw_sum;
        shm_data.max_raw = motion.max_raw;
        shm_data.min_raw = motion.min_raw;
        shm_data.shutter_upper = motion.shutter_upper;
        shm_data.shutter_lower = motion.shutter_lower;
    }

    shm_publish_flow(&shm_data);
}

static void sensor_publish_tof (TickType_t now_tick)
{
    driver_vl53l1x_data_struct data = {0};
    shm_tof_data_struct shm_data;
    uint8 ret = 0U;

    memset(&shm_data, 0, sizeof(shm_data));
    ret = driver_vl53l1x_read(&data);

    shm_data.timestamp_ms = sensor_tick_to_ms(now_tick);
    shm_data.healthy = (ret == 0U) ? 1U : 0U;
    shm_data.error_code = ret;
    sensor_fill_tof_diag(&shm_data);

    if (ret == 0U)
    {
        shm_data.distance_mm = data.distance_mm;
        shm_data.range_status = data.range_status;
        shm_data.data_ready = data.data_ready;
    }

    shm_publish_tof(&shm_data);
}

static void imu_task_entry (void *parameter)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_init_try = 0U;
    uint8 imu_inited = 0U;

    (void)parameter;

    while (1)
    {
        TickType_t now_tick = xTaskGetTickCount();

        if ((!imu_inited) &&
            ((last_init_try == 0U) ||
             ((sensor_tick_to_ms(now_tick) - sensor_tick_to_ms(last_init_try)) >= SENSOR_OPTIONAL_RETRY_MS)))
        {
            last_init_try = now_tick;
            imu_inited = (driver_sch16tk01_init() == 0U) ? 1U : 0U;
            if (imu_inited)
            {
                sch16tk01_calibrate_gyro_bias();
            }
        }

        if (imu_inited)
        {
            if (sensor_publish_imu(now_tick) == 2U)
            {
                imu_inited = 0U;
                sensor_publish_imu_not_ready(now_tick, 2U);
            }
        }
        else
        {
            sensor_publish_imu_not_ready(now_tick, 0xE0U);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_IMU_PERIOD_MS));
    }
}

static void flow_task_entry (void *parameter)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_init_try = 0U;
    uint8 flow_inited = 0U;
    uint8 flow_init_ret = 0U;

    (void)parameter;

    while (1)
    {
        TickType_t now_tick = xTaskGetTickCount();
        uint8 flow_enabled = flight_param_get_u8_default(FLIGHT_PARAM_FLOW_ENABLE, 1U);

        if (!flow_enabled)
        {
            flow_inited = 0U;
            sensor_publish_optional_disabled_flow(now_tick);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_FLOW_PERIOD_MS));
            continue;
        }

        if ((!flow_inited) &&
            ((last_init_try == 0U) ||
             ((sensor_tick_to_ms(now_tick) - sensor_tick_to_ms(last_init_try)) >= SENSOR_OPTIONAL_RETRY_MS)))
        {
            last_init_try = now_tick;
            flow_init_ret = driver_pmw3901_init();
            flow_inited = (flow_init_ret == 0U) ? 1U : 0U;
        }

        if (flow_inited)
        {
            sensor_publish_flow(now_tick);
        }
        else
        {
            shm_flow_data_struct shm_data;

            memset(&shm_data, 0, sizeof(shm_data));
            shm_data.timestamp_ms = sensor_tick_to_ms(now_tick);
            shm_data.healthy = 0U;
            shm_data.error_code = (flow_init_ret != 0U) ? flow_init_ret : 0xE1U;
            sensor_fill_flow_diag(&shm_data);
            shm_publish_flow(&shm_data);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_FLOW_PERIOD_MS));
    }
}

static void tof_task_entry (void *parameter)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_init_try = 0U;
    uint8 tof_inited = 0U;
    uint8 tof_init_ret = 0U;

    (void)parameter;

    while (1)
    {
        TickType_t now_tick = xTaskGetTickCount();
        uint8 tof_enabled = flight_param_get_u8_default(FLIGHT_PARAM_TOF_ENABLE, 1U);

        if (!tof_enabled)
        {
            tof_inited = 0U;
            sensor_publish_optional_disabled_tof(now_tick);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_TOF_PERIOD_MS));
            continue;
        }

        if ((!tof_inited) &&
            ((last_init_try == 0U) ||
             ((sensor_tick_to_ms(now_tick) - sensor_tick_to_ms(last_init_try)) >= SENSOR_OPTIONAL_RETRY_MS)))
        {
            last_init_try = now_tick;
            tof_init_ret = driver_vl53l1x_init();
            tof_inited = (tof_init_ret == 0U) ? 1U : 0U;
        }

        if (tof_inited)
        {
            sensor_publish_tof(now_tick);
        }
        else
        {
            shm_tof_data_struct shm_data;

            memset(&shm_data, 0, sizeof(shm_data));
            shm_data.timestamp_ms = sensor_tick_to_ms(now_tick);
            shm_data.healthy = 0U;
            shm_data.error_code = (tof_init_ret != 0U) ? tof_init_ret : 0xE1U;
            sensor_fill_tof_diag(&shm_data);
            shm_publish_tof(&shm_data);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_TOF_PERIOD_MS));
    }
}

void tasks_sensor_init (void)
{
    if (g_imu_task_handle == 0)
    {
        xTaskCreate(imu_task_entry, SENSOR_IMU_TASK_NAME, SENSOR_IMU_TASK_STACK_WORDS, 0, SENSOR_IMU_TASK_PRIORITY, &g_imu_task_handle);
    }

    if (g_flow_task_handle == 0)
    {
        xTaskCreate(flow_task_entry, SENSOR_FLOW_TASK_NAME, SENSOR_FLOW_TASK_STACK_WORDS, 0, SENSOR_FLOW_TASK_PRIORITY, &g_flow_task_handle);
    }

    if (g_tof_task_handle == 0)
    {
        xTaskCreate(tof_task_entry, SENSOR_TOF_TASK_NAME, SENSOR_TOF_TASK_STACK_WORDS, 0, SENSOR_TOF_TASK_PRIORITY, &g_tof_task_handle);
    }
}
