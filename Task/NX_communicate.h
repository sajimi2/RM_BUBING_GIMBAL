#ifndef __NX_COMMUNICATE_H
#define __NX_COMMUNICATE_H

#include "struct_typedef.h"

// 上位机 -> MCU
typedef struct __attribute__((packed))
{
    uint8_t mode;    // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    int16_t yaw;   // 绝对目标角：板端解码为 yaw * PI / 32767，须与 INS.Yaw 同参考系
    int16_t pitch; // 绝对目标角：板端解码为 -pitch * PI / 32767，须与 INS.Pitch 同参考系
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
