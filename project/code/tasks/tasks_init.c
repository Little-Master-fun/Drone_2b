#include "tasks.h"

#include "config/flight_params.h"
#include "config/shared_memory.h"

void tasks_init_all (void)
{
    shm_sensors_init();
    flight_params_init();

    tasks_sensor_init();
    tasks_estimator_init();
    tasks_commander_init();
    tasks_control_init();
    tasks_mixer_init();
    tasks_comm_init();
}
