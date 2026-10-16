#ifndef ROBOT_TOTAL_MODE_H
#define ROBOT_TOTAL_MODE_H
#include "struct_typedef.h"

extern int robotIsAuto(void);  // 对外提供的接口，返回目前机器人是否自动
const enum RobotState_e * getRobotPresentMode(void);  //返回机器人当前状态指针
void robot_total_mode_task(void const *pvParameters);
void CtrlModeDetect(void);

#endif
