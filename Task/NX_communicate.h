#ifndef __NX_COMMUNICATE_H
#define __NX_COMMUNICATE_H

#include "struct_typedef.h"

// 上位机 -> MCU
typedef struct __attribute__((packed))
{
    uint8_t head[2]; // 'S', 'P'
    uint8_t mode;    // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} VisionToGimbal_t;

// MCU -> 上位机
typedef struct __attribute__((packed))
{
    uint8_t head[2]; // 'S', 'P'
    uint8_t mode;    // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4];      // wxyz顺序
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count; // 子弹累计发送次数
    uint16_t crc16;
} GimbalToVision_t;

extern VisionToGimbal_t NX_data;
extern volatile uint8_t data_ready;

// 初始化函数
void NXrecieve_Init(void);
const VisionToGimbal_t* get_nx_control_point(void);
void NX_task(void const *pvParameters);

#endif
