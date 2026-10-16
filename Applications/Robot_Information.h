/**
 * @file Robot_Information.h
 * @brief 通用的数据结构
 * @Auther Jing Yuhang  jyh16213839418@163.com
 * @version 0.3
 * @date 2025-12-21
 *
 */
#ifndef ROBOT_INFORMATION_H
#define ROBOT_INFORMATION_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan1
#define SHOOT_CAN hcan2

typedef enum
{
    
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    CAN_SHOOT_ALL_ID=0X200,
    CAN_SHOOT_3508_ID_1=0X201,
    CAN_SHOOT_3508_ID_2=0X202,
    // 两个摩擦轮3508挂载在云台can2上
} can_msg_id_e;

typedef struct							//所有电机数据结构
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

enum RobotState_e{						//整车模式枚举，将totalmode整合进information
    RobotState_e_Powerless=0,     		//无力模式

    RobotState_e_CommonCar=3,     		//普通车模式

    RobotState_e_GimbalCar=1,        	//底盘跟随云台
                                
    RobotState_e_Spinner=2,     		//小陀螺
};
#endif
