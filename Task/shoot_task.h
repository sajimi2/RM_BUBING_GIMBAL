#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "pid.h"
#include "remote_control.h"
#include "Robot_Information.h"
#include "robot_total_mode.h"
#include "CAN_recieve_send.h"
#include "INS_task.h"
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 任务时长常量 ------------------------------------------------------------*/
#define SHOOT_TASK_INIT_TIME 500
#define SHOOT_CTRL_TIME 5

/* 拨弹参数 ----------------------------------------------------------------*/
#define TRIGGER_ROUNDS_FOR_A_BULLET_A 4
#define TRIGGER_ROUNDS_FOR_A_BULLET_B 4
#define TRIGGER_ROUNDS_FOR_A_BULLET_C 4
#define ECD_FULL_ROUND 8192
#define DEAD_BOND 5

/* 电机速度环PID参数 ------------------------------------------------------*/
#define M3508_MOTOR_SPEED_PID_KP 60000.0f
#define M3508_MOTOR_SPEED_PID_KI 0.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f

#define M2006_MOTOR_SPEED_PID_KP 20000.0f
#define M2006_MOTOR_SPEED_PID_KI 0.0f
#define M2006_MOTOR_SPEED_PID_KD 1000.0f

/* 电机电流限制 ------------------------------------------------------------*/
#define MAX_MOTOR_3508_CURRENT 30000.0f
#define MAX_MOTOR_2006_CURRENT 30000.0f

#define M3505_MOTOR_SHOOT_SPEED_PID_MAX_OUT    MAX_MOTOR_3508_CURRENT
#define M3505_MOTOR_SHOOT_SPEED_PID_MAX_IOUT   2000.0f

#define M2006_MOTOR_SPEED_PID_MAX_OUT          MAX_MOTOR_2006_CURRENT
#define M2006_MOTOR_SPEED_PID_MAX_IOUT         2000.0f

/* RPM转速度向量系数 ------------------------------------------------------*/
#define SHOOT_M3508_MOTOR_RPM_TO_VECTOR (0.000415809748903494517209f/5)
#define SHOOT_M2006_MOTOR_RPM_TO_VECTOR (0.000415809748903494517209f/5)

/* 里程碑栈法参数 ----------------------------------------------------------*/
#define MILESTONE_NEAR_THRESHHOLD 500
#define MILESTONE_NUMBER 3


extern bool_t fricOn;        // 摩擦轮开启标志
extern uint8_t triggerOn;    // 拨弹轮开启标志
extern int16_t giveShootCurrent[2];  // 摩擦轮输出电流
extern int16_t giveTriggerCurrent;   // 拨弹轮输出电流

/**
 * @brief 发射任务主函数
 * @param pvParameters 任务参数
 */
void shoot_task(void const *pvParameters);

/**
 * @brief 发射任务触发器监控函数
 * @param argument 任务参数
 */
void shootTaskTrggMonitor(void const *argument);

/**
 * @brief 获取拨弹轮电流指针
 * @return 指向拨弹轮电流的指针
 */
int16_t * getTriggerCurrentP(void);

/**
 * @brief 获取摩擦轮状态
 * @return 摩擦轮开启状态
 */
bool_t get_fric(void);

/* 内部函数声明（供其他模块使用）------------------------------------------*/

/**
 * @brief 获取摩擦轮电机测量点
 * @param motor_id 电机ID (0或1)
 * @return 电机测量数据结构指针
 */
const motor_measure_t *get_shoot_motor_measure_point(uint8_t motor_id);

/**
 * @brief 获取拨弹轮电机测量点
 * @return 电机测量数据结构指针
 */
const motor_measure_t *get_trigger_motor_measure_point(void);

/**
 * @brief 发送射击控制CAN命令
 * @param current1 电机1电流
 * @param current2 电机2电流
 */
void CAN_cmd_shoot(int16_t current1, int16_t current2);

/**
 * @brief 获取遥控器数据指针
 * @return 遥控器数据结构指针
 */
const RC_ctrl_t *get_remote_control_point(void);

/**
 * @brief 获取机器人是否处于自动模式
 * @return 自动模式状态
 */
int robotIsAuto(void);

#ifdef __cplusplus
}
#endif

#endif /* __SHOOT_TASK_H */
