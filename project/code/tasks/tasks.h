#ifndef _TASKS_H_
#define _TASKS_H_

#ifdef __cplusplus
extern "C" {
#endif

void tasks_sensor_init(void);
void tasks_estimator_init(void);
void tasks_commander_init(void);
void tasks_control_init(void);
void tasks_mixer_init(void);
void tasks_comm_init(void);
void tasks_init_all(void);

#ifdef __cplusplus
}
#endif

#endif
