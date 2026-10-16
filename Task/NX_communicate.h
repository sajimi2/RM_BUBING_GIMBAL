#ifndef __NX_COMMUNICATE_H
#define __NX_COMMUNICATE_H

#include "struct_typedef.h"

// 上位机 -> MCU：现有 Tracker -> mpc_can_sender 的标准 CAN 0x005、DLC=8、小端报文。
// 视觉发送端完成方向转换；板端将两轴有符号整数直接解码为 INS 同向绝对弧度。
typedef struct __attribute__((packed))
{
    uint8_t mode;    // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    int16_t yaw;   // 绝对目标角：板端解码为 yaw * PI / 32767，须与 INS.Yaw 同参考系
    int16_t pitch; // 绝对目标角：板端解码为 pitch * PI / 32767，须与 INS.Pitch 同参考系
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
	
/* 中断写入、任务读取；任务须用 NX_TakeControlFrame 获取完整的新帧。 */
extern volatile CAN_Send_to_MCU NX_data;
extern volatile uint8_t data_ready;

/* 调试时展开 vision_debug：区分报文未变、目标未采用和闭环未跟随。
 * 正负计数及最后原始值会保留，便于一次测试后查看两种输入是否都到板。
 */
typedef struct
{
    uint32_t rx_count;
    uint32_t applied_count;
    uint32_t positive_count;
    uint32_t negative_count;
    int16_t rx_yaw_raw;
    int16_t last_positive_raw;
    int16_t last_negative_raw;
    int16_t applied_yaw_raw;
    fp32 yaw_target_rad;
    fp32 yaw_feedback_rad;
    fp32 yaw_error_rad;
    int16_t yaw_output;
    uint8_t auto_active;
    uint8_t remote_online;
    uint8_t rx_bus;
    uint8_t rx_bytes[8];
} VisionDebug_t;
extern volatile VisionDebug_t vision_debug;

void NXrecieve_Init(void);
void Send_Gimbal_Information(void);
const volatile CAN_Send_to_MCU* get_nx_control_point(void);
uint8_t NX_TakeControlFrame(CAN_Send_to_MCU *frame);
#endif
