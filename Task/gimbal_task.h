#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H


#include "struct_typedef.h"

struct GimbalToChassis_s{
    int8_t effective;   //数据有效
    fp32 w;     //角速度，rad/s
};

void gimbal_task(void const *pvParameters);

const struct GimbalToChassis_s * getBadYawWantedSpd(void);
const int16_t * getGimbalNowRoundsPoint(void);
extern const struct GimbalMotorCtrl_s* get_wantedyaw_point(void);
extern const struct GimbalMotorCtrl_s* get_wantedpitch_point(void);
fp32 get_yaw_current_angle(void);
fp32 get_yaw_current_speed(void);
fp32 get_pitch_current_angle(void);
fp32 get_pitch_current_speed(void);
#endif
