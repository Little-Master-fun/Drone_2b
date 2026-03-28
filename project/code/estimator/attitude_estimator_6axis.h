#ifndef _attitude_estimator_6axis_h_
#define _attitude_estimator_6axis_h_

#include "zf_common_typedef.h"

typedef struct
{
    float q[4];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float accel_norm;
    uint8 valid;
} attitude_estimator_6axis_state_struct;

void attitude_estimator_6axis_init(void);
void attitude_estimator_6axis_reset(void);
uint8 attitude_estimator_6axis_update(float gx_rad_s,
                                      float gy_rad_s,
                                      float gz_rad_s,
                                      float ax_mss,
                                      float ay_mss,
                                      float az_mss,
                                      float dt_s);
attitude_estimator_6axis_state_struct attitude_estimator_6axis_get_state(void);

#endif
