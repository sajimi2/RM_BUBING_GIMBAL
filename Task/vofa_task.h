#ifndef VOFA_TASK_H
#define VOFA_TASK_H

/* 独立处理调参串口；任务优先级低于云台和IMU，不参与电机控制计算。 */
void vofa_task(void const *argument);

#endif
