#ifndef _SHARED_MEMORY_H_
#define _SHARED_MEMORY_H_

#include "zf_common_typedef.h"

#define SHM_SENSOR_DETECT_UNKNOWN        (0U)
#define SHM_SENSOR_DETECT_OK             (1U)
#define SHM_SENSOR_DETECT_WIRING_FAULT   (2U)
#define SHM_SENSOR_DETECT_CHIP_MISMATCH  (3U)
#define SHM_SENSOR_DETECT_INIT_FAILED    (4U)

#define SHM_TOF_DRIVER_UNKNOWN           (0U)
#define SHM_TOF_DRIVER_VL53L0X           (1U)
#define SHM_TOF_DRIVER_VL53L1X           (2U)

typedef struct
{
    uint32 timestamp_ms;
    uint8 healthy;
    uint8 error_code;
    int16 accel_x_raw;
    int16 accel_y_raw;
    int16 accel_z_raw;
    int16 gyro_x_raw;
    int16 gyro_y_raw;
    int16 gyro_z_raw;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} shm_imu_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 healthy;
    uint8 error_code;
    int16 mag_x_raw;
    int16 mag_y_raw;
    int16 mag_z_raw;
    float mag_x_gauss;
    float mag_y_gauss;
    float mag_z_gauss;
} shm_mag_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 healthy;
    uint8 error_code;
    float temperature_c;
    float pressure_pa;
} shm_baro_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 healthy;
    uint8 error_code;
    uint8 detect_status;
    uint8 expected_chip_id;
    uint8 detected_chip_id;
    uint8 detected_chip_id_inverse;
    int16 delta_x;
    int16 delta_y;
    uint8 motion;
    uint8 squal;
    uint8 raw_sum;
    uint8 max_raw;
    uint8 min_raw;
    uint8 shutter_upper;
    uint8 shutter_lower;
} shm_flow_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 healthy;
    uint8 error_code;
    uint8 detect_status;
    uint8 driver_type;
    uint8 detected_type;
    uint8 expected_model_id;
    uint8 detected_model_id;
    uint8 detected_alt_model_id;
    uint16 distance_mm;
    uint8 range_status;
    uint8 data_ready;
} shm_tof_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 flight_mode;
    uint8 armed;
    uint8 failsafe;
    uint8 sensors_healthy;
    uint8 nav_state;
    uint8 failsafe_reason;
    uint8 land_state;
} shm_commander_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 healthy;
    uint8 flow_fused;
    uint8 active_imu;
    uint8 altitude_source;
    uint8 accel_fused;
    uint8 alt_rejected;
    uint8 flow_rejected;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float altitude_m;
    float vertical_speed_mps;
    float pos_x_m;
    float pos_y_m;
    float vel_x_mps;
    float vel_y_mps;
} shm_estimator_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 armed;
    uint8 failsafe;
    uint8 telemetry_flags;
    float throttle;
    float roll_cmd;
    float pitch_cmd;
    float yaw_cmd;
    float altitude_sp_m;
    float vel_sp_x_mps;
    float vel_sp_y_mps;
    float vel_sp_z_mps;
    float alt_i_term;
    float roll_sp_rad;
    float pitch_sp_rad;
    float rate_sp_roll_rps;
    float rate_sp_pitch_rps;
    float rate_sp_yaw_rps;
    float rate_meas_roll_rps;
    float rate_meas_pitch_rps;
    float rate_meas_yaw_rps;
    uint8 hover_thrust_status;
    uint8 hover_thrust_valid;
    float hover_thrust_est;
    float hover_thrust_var;
    float hover_thrust_innov;
    float hover_thrust_innov_var;
    float hover_thrust_test_ratio;
    float hover_thrust_accel_mps2;
    float hover_thrust_accel_noise;
} shm_control_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 enabled;
    uint8 saturation_high;
    uint8 saturation_low;
    uint8 yaw_limited;
    uint8 thrust_reduced;
    uint16 motor1;
    uint16 motor2;
    uint16 motor3;
    uint16 motor4;
    float yaw_scale;
    float collective_offset;
} shm_mixer_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 link_up;
    uint32 tx_count;
    uint32 rx_count;
} shm_comm_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 link_active;
    uint8 arm_request;
    uint8 control_active;
    uint8 land_request;
    uint8 estop_request;
    float forward;
    float right;
    float up;
    float yaw;
} shm_host_control_data_struct;

typedef struct
{
    uint32 timestamp_ms;
    uint8 sd_ready;
    uint32 log_counter;
    uint32 error_counter;
} shm_logging_data_struct;

void shm_sensors_init(void);

void shm_publish_imu(uint8 imu_index, const shm_imu_data_struct *data);
uint8 shm_read_imu(uint8 imu_index, shm_imu_data_struct *data, uint32 *seq);

void shm_publish_mag(const shm_mag_data_struct *data);
uint8 shm_read_mag(shm_mag_data_struct *data, uint32 *seq);

void shm_publish_baro(const shm_baro_data_struct *data);
uint8 shm_read_baro(shm_baro_data_struct *data, uint32 *seq);

void shm_publish_flow(const shm_flow_data_struct *data);
uint8 shm_read_flow(shm_flow_data_struct *data, uint32 *seq);

void shm_publish_tof(const shm_tof_data_struct *data);
uint8 shm_read_tof(shm_tof_data_struct *data, uint32 *seq);

void shm_publish_commander(const shm_commander_data_struct *data);
uint8 shm_read_commander(shm_commander_data_struct *data, uint32 *seq);

void shm_publish_estimator(const shm_estimator_data_struct *data);
uint8 shm_read_estimator(shm_estimator_data_struct *data, uint32 *seq);

void shm_publish_control(const shm_control_data_struct *data);
uint8 shm_read_control(shm_control_data_struct *data, uint32 *seq);

void shm_publish_mixer(const shm_mixer_data_struct *data);
uint8 shm_read_mixer(shm_mixer_data_struct *data, uint32 *seq);

void shm_publish_comm(const shm_comm_data_struct *data);
uint8 shm_read_comm(shm_comm_data_struct *data, uint32 *seq);

void shm_publish_host_control(const shm_host_control_data_struct *data);
uint8 shm_read_host_control(shm_host_control_data_struct *data, uint32 *seq);

void shm_publish_logging(const shm_logging_data_struct *data);
uint8 shm_read_logging(shm_logging_data_struct *data, uint32 *seq);

#endif
