#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "shared_memory.h"

typedef struct
{
    uint32 seq;
    shm_imu_data_struct data;
} shm_imu_slot_struct;

typedef struct
{
    uint32 seq;
    shm_mag_data_struct data;
} shm_mag_slot_struct;

typedef struct
{
    uint32 seq;
    shm_baro_data_struct data;
} shm_baro_slot_struct;

typedef struct
{
    uint32 seq;
    shm_flow_data_struct data;
} shm_flow_slot_struct;

typedef struct
{
    uint32 seq;
    shm_tof_data_struct data;
} shm_tof_slot_struct;

typedef struct
{
    uint32 seq;
    shm_commander_data_struct data;
} shm_commander_slot_struct;

typedef struct
{
    uint32 seq;
    shm_estimator_data_struct data;
} shm_estimator_slot_struct;

typedef struct
{
    uint32 seq;
    shm_control_data_struct data;
} shm_control_slot_struct;

typedef struct
{
    uint32 seq;
    shm_mixer_data_struct data;
} shm_mixer_slot_struct;

typedef struct
{
    uint32 seq;
    shm_comm_data_struct data;
} shm_comm_slot_struct;

typedef struct
{
    uint32 seq;
    shm_host_control_data_struct data;
} shm_host_control_slot_struct;

typedef struct
{
    uint32 seq;
    shm_logging_data_struct data;
} shm_logging_slot_struct;

typedef struct
{
    shm_imu_slot_struct imu[2];
    shm_mag_slot_struct mag;
    shm_baro_slot_struct baro;
    shm_flow_slot_struct flow;
    shm_tof_slot_struct tof;
    shm_commander_slot_struct commander;
    shm_estimator_slot_struct estimator;
    shm_control_slot_struct control;
    shm_mixer_slot_struct mixer;
    shm_comm_slot_struct comm;
    shm_host_control_slot_struct host_control;
    shm_logging_slot_struct logging;
} shm_storage_struct;

static shm_storage_struct g_shm = {0};

static uint8 shm_valid_imu_index (uint8 imu_index)
{
    return (imu_index < 2U) ? 1U : 0U;
}

#define SHM_PUBLISH(slot_name, data_ptr)            \
    do                                              \
    {                                               \
        if ((data_ptr) != 0)                        \
        {                                           \
            taskENTER_CRITICAL();                   \
            g_shm.slot_name.data = *(data_ptr);     \
            g_shm.slot_name.seq++;                  \
            taskEXIT_CRITICAL();                    \
        }                                           \
    } while (0)

#define SHM_READ(slot_name, data_ptr, seq_ptr)      \
    do                                              \
    {                                               \
        if ((data_ptr) == 0)                        \
        {                                           \
            return 1U;                              \
        }                                           \
        taskENTER_CRITICAL();                       \
        *(data_ptr) = g_shm.slot_name.data;         \
        if ((seq_ptr) != 0)                         \
        {                                           \
            *(seq_ptr) = g_shm.slot_name.seq;       \
        }                                           \
        taskEXIT_CRITICAL();                        \
        return 0U;                                  \
    } while (0)

void shm_sensors_init (void)
{
    taskENTER_CRITICAL();
    memset(&g_shm, 0, sizeof(g_shm));
    taskEXIT_CRITICAL();
}

void shm_publish_imu (uint8 imu_index, const shm_imu_data_struct *data)
{
    if ((!shm_valid_imu_index(imu_index)) || (data == 0))
    {
        return;
    }

    taskENTER_CRITICAL();
    g_shm.imu[imu_index].data = *data;
    g_shm.imu[imu_index].seq++;
    taskEXIT_CRITICAL();
}

uint8 shm_read_imu (uint8 imu_index, shm_imu_data_struct *data, uint32 *seq)
{
    if ((!shm_valid_imu_index(imu_index)) || (data == 0))
    {
        return 1U;
    }

    taskENTER_CRITICAL();
    *data = g_shm.imu[imu_index].data;
    if (seq != 0)
    {
        *seq = g_shm.imu[imu_index].seq;
    }
    taskEXIT_CRITICAL();
    return 0U;
}

void shm_publish_mag (const shm_mag_data_struct *data)
{
    SHM_PUBLISH(mag, data);
}

uint8 shm_read_mag (shm_mag_data_struct *data, uint32 *seq)
{
    SHM_READ(mag, data, seq);
}

void shm_publish_baro (const shm_baro_data_struct *data)
{
    SHM_PUBLISH(baro, data);
}

uint8 shm_read_baro (shm_baro_data_struct *data, uint32 *seq)
{
    SHM_READ(baro, data, seq);
}

void shm_publish_flow (const shm_flow_data_struct *data)
{
    SHM_PUBLISH(flow, data);
}

uint8 shm_read_flow (shm_flow_data_struct *data, uint32 *seq)
{
    SHM_READ(flow, data, seq);
}

void shm_publish_tof (const shm_tof_data_struct *data)
{
    SHM_PUBLISH(tof, data);
}

uint8 shm_read_tof (shm_tof_data_struct *data, uint32 *seq)
{
    SHM_READ(tof, data, seq);
}

void shm_publish_commander (const shm_commander_data_struct *data)
{
    SHM_PUBLISH(commander, data);
}

uint8 shm_read_commander (shm_commander_data_struct *data, uint32 *seq)
{
    SHM_READ(commander, data, seq);
}

void shm_publish_estimator (const shm_estimator_data_struct *data)
{
    SHM_PUBLISH(estimator, data);
}

uint8 shm_read_estimator (shm_estimator_data_struct *data, uint32 *seq)
{
    SHM_READ(estimator, data, seq);
}

void shm_publish_control (const shm_control_data_struct *data)
{
    SHM_PUBLISH(control, data);
}

uint8 shm_read_control (shm_control_data_struct *data, uint32 *seq)
{
    SHM_READ(control, data, seq);
}

void shm_publish_mixer (const shm_mixer_data_struct *data)
{
    SHM_PUBLISH(mixer, data);
}

uint8 shm_read_mixer (shm_mixer_data_struct *data, uint32 *seq)
{
    SHM_READ(mixer, data, seq);
}

void shm_publish_comm (const shm_comm_data_struct *data)
{
    SHM_PUBLISH(comm, data);
}

uint8 shm_read_comm (shm_comm_data_struct *data, uint32 *seq)
{
    SHM_READ(comm, data, seq);
}

void shm_publish_host_control (const shm_host_control_data_struct *data)
{
    SHM_PUBLISH(host_control, data);
}

uint8 shm_read_host_control (shm_host_control_data_struct *data, uint32 *seq)
{
    SHM_READ(host_control, data, seq);
}

void shm_publish_logging (const shm_logging_data_struct *data)
{
    SHM_PUBLISH(logging, data);
}

uint8 shm_read_logging (shm_logging_data_struct *data, uint32 *seq)
{
    SHM_READ(logging, data, seq);
}
