#include "bsp_usart.h"
#include "NX_communicate.h"
#include "Robot_Information.h"
#include "usart.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "robot_total_mode.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_Send_to_MCU NX_data;
CAN_send_to_NX data_u;

static CAN_TxHeaderTypeDef  CAN2_Send_to_NX;
Send_Quaternion u16_Quaternion;

INS_t *Raw_Quaternion;

volatile uint8_t data_ready = 0;

void NXrecieve_Init(void)
{
    Raw_Quaternion = get_INS();
	memset(&data_u,0,sizeof(data_u));
	
	memset(&NX_data,0,sizeof(NX_data));
}

void Q_reflection(INS_t *q_data)
{
	data_u.u16_Quaternion.qw = Raw_Quaternion->q[0] * 32767;
	data_u.u16_Quaternion.qx = Raw_Quaternion->q[1] * 32767;
	data_u.u16_Quaternion.qy = Raw_Quaternion->q[2] * 32767;
	data_u.u16_Quaternion.qz = Raw_Quaternion->q[3] * 32767;
}

void Send_Gimbal_Information(void)
{
    uint32_t mail_box;
	
	Q_reflection(Raw_Quaternion);

	CAN2_Send_to_NX.StdId = 0x006;
	CAN2_Send_to_NX.IDE   = CAN_ID_STD;
	CAN2_Send_to_NX.RTR   = CAN_RTR_DATA;
	CAN2_Send_to_NX.DLC   = 0x08;
	
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_Send_to_NX, data_u.data, &mail_box);
}

const CAN_Send_to_MCU *get_nx_control_point(void)
{
    return &NX_data;
}
