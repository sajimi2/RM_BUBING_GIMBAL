#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H


#include "struct_typedef.h"

void gimbal_task(void const *pvParameters);
extern const struct GimbalMotorCtrl_s* get_wantedyaw_point(void);
extern const struct GimbalMotorCtrl_s* get_wantedpitch_point(void);
fp32 get_yaw_current_angle(void);
fp32 get_yaw_current_speed(void);
fp32 get_pitch_current_angle(void);
fp32 get_pitch_current_speed(void);
#endif
