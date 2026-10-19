#ifndef __NX_COMMUNICATE_H
#define __NX_COMMUNICATE_H

#include "struct_typedef.h"

// 上位机 -> MCU
typedef struct __attribute__((packed))//后缀是GCC扩展，表示结构体成员紧密排列
{
    uint8_t mode;    // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    int16_t yaw;
    int16_t pitch;
	uint8_t fire;
	int16_t rev;
} CAN_Send_to_MCU;

// MCU -> 上位机
typedef struct
{
	int16_t qw;
	int16_t qx;
	int16_t qy;
	int16_t qz;
}Send_Quaternion;

typedef union
{
	Send_Quaternion u16_Quaternion;
	uint8_t data[8];
}CAN_send_to_NX;
	
extern CAN_Send_to_MCU NX_data;
extern volatile uint8_t data_ready;

void NXrecieve_Init(void);
void Send_Gimbal_Information(void);
const CAN_Send_to_MCU* get_nx_control_point(void);
#endif
