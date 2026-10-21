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
static volatile uint8_t input_source = NX_SOURCE_CAN;
extern uint8_t fire_t;

uint8_t NX_GetSource(void)
{
    return input_source;
}

void NX_SelectSource(uint8_t source)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    if (source != input_source)
    {
        input_source = source;
        data_ready = 0;
        fire_t = 0;
        NX_data.fire = 0;// 清除旧的开火请求，避免切换来源后误触发
    }
    __set_PRIMASK(key);
}

uint8_t NX_ReceiveFrame(const uint8_t payload[8], uint8_t source)
{
    CAN_Send_to_MCU frame;
    uint32_t key;
    uint8_t accepted = 0;
    frame.mode = payload[0];
    frame.yaw = (int16_t)((uint16_t)payload[1] | ((uint16_t)payload[2] << 8));
    frame.pitch = (int16_t)((uint16_t)payload[3] | ((uint16_t)payload[4] << 8));
    frame.fire = payload[5];
    frame.rev = (int16_t)((uint16_t)payload[6] | ((uint16_t)payload[7] << 8));

    /* 串口任务和 CAN 中断都能发布数据；短临界区保证来源与整帧一起更新。 */
    key = __get_PRIMASK();
    __disable_irq();
    if (source == input_source)
    {
        NX_data = frame;
        fire_t = frame.fire;
        data_ready = 1;
        accepted = 1;
    }
    __set_PRIMASK(key);
    return accepted;
}

uint8_t NX_TakeControlFrame(CAN_Send_to_MCU *frame, uint8_t *source)
{
    uint8_t ready;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    ready = data_ready;
    if (ready)
    {
        *frame = NX_data;
        *source = input_source;
        data_ready = 0;
    }
    __set_PRIMASK(key);
    return ready;
}

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
