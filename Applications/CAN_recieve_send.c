#include "main.h"
#include "can.h"
#include "gpio.h"
#include "CAN_recieve_send.h"
#include "robot_total_mode.h"
#include "INS_task.h"	
#include "remote_control.h"
#include "gimbal_task.h"
#include "user_lib.h"
#include "pid.h"
#include "Robot_Information.h"
#include "NX_communicate.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern volatile uint8_t  rc_online;     // 遥控器在线状态：0-离线，1-在线
extern volatile uint8_t data_ready;
extern uint8_t fire_t;

//用于存放所有信息。提供函数接口供外界使用
static motor_measure_t motor_chassis[7];//四个底盘3508，一个拨弹，一个yaw，一个pitch
static motor_measure_t motor_shoot[2];
static volatile gimbal_rx_observation_t gimbal_rx_observation;

const volatile gimbal_rx_observation_t *get_gimbal_rx_observation(void)
{
    return &gimbal_rx_observation;
}

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];

static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];

extern CAN_Send_to_MCU NX_data;

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//收取回调，理论上云台c板不会管底盘电机的数据的，应该收拨弹和yaw的数据
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_3508_M1_ID;
			if((hcan==&SHOOT_CAN)&&(i==0||i==1))
			{
				get_motor_measure(&motor_shoot[i],rx_data);//摩擦轮状态
				break;
			}		
			get_motor_measure(&motor_chassis[i], rx_data);//一个拨弹，一个yaw，一个pitch

            /* 仅记录收包时间；掉线时 VOFA 显示数据年龄，避免把旧角度当新反馈。 */
            if (i == 4)
            {
                gimbal_rx_observation.yaw_last_ms = HAL_GetTick();
                gimbal_rx_observation.yaw_seen = 1;
            }
            else if (i == 5)
            {
                gimbal_rx_observation.pitch_last_ms = HAL_GetTick();
                gimbal_rx_observation.pitch_seen = 1;
            }
			break;
		}
		case 0x004://裁判系统发的东西，先不管
		{
//			typedef union
//				{
//					can_send_referee_s data_s;
//					uint8_t data_uint8_referee[sizeof(can_send_referee_s)];
//				}data_u_referee;
//				data_u_referee data_0;
//				for(int i=0;i<sizeof(can_send_referee_s);i++){
//					data_0.data_uint8_referee[i]=rx_data[i];
//				}
//				gimbal_rx_referee.shootspeed=data_0.data_s.shootspeed;
//				gimbal_rx_referee.robot_id=data_0.data_s.robot_id;
			//获取射速
				break;
		}
		case 0x005:
{
    typedef union
    {
        CAN_Send_to_MCU NX_data;
        uint8_t data[8];
    } can_recieve_nx_data;

    can_recieve_nx_data data_u;
    memset(&data_u, 0, sizeof(data_u));

    // 手动按小端序解析 yaw（第2、3字节）
    data_u.NX_data.yaw = (int16_t)(
        ((uint16_t)rx_data[1] << 0) |  // 低8位
        ((uint16_t)rx_data[2] << 8)    // 高8位
    );

    // 手动按小端序解析 pitch（第4、5字节）
    data_u.NX_data.pitch = (int16_t)(
        ((uint16_t)rx_data[3] << 0) |  // 低8位
        ((uint16_t)rx_data[4] << 8)    // 高8位
    );

    // 其他字节按需拷贝
    data_u.data[0] = rx_data[0];
    data_u.data[5] = rx_data[5];
    data_u.data[6] = rx_data[6];
    data_u.data[7] = rx_data[7];

    NX_data = data_u.NX_data;
    gimbal_rx_observation.vision_last_ms = HAL_GetTick();
    gimbal_rx_observation.vision_seen = 1;
    data_ready = 1;
	
	fire_t = NX_data.fire;
    break;
}
        default:
        {
            break;
        }
    }
}
/**
  * @brief          发送云台电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
void CAN_cmd_yaw(int16_t yaw)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = 0;
    gimbal_can_send_data[3] = 0;
    gimbal_can_send_data[4] = 0;
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}
static const RC_ctrl_t *rc_chassis;

static CAN_TxHeaderTypeDef  can1_tx_channel_message;

//这个通道发送底盘控制量
typedef struct
{
	int16_t channel_2;
	int16_t channel_3;
	uint8_t mode;
	uint8_t onlineflag;
	uint16_t key_v;    // 键盘按键值
}can_send_encode_data_s;
void CAN1_send_channel()
{
    uint32_t send_mail_box;
    
    typedef union {
        can_send_encode_data_s data;
        uint8_t data_1[8];
    } can_send_data_channel_u;
    
    can_send_data_channel_u data_u;
    memset(&data_u, 0, sizeof(data_u));
    
    // 获取遥控数据
    rc_chassis = get_remote_control_point();
	
	data_u.data.onlineflag= rc_online;
    data_u.data.mode      = (uint8_t)*getRobotPresentMode();
    data_u.data.channel_2 = rc_chassis->rc.ch[2];
    data_u.data.channel_3 = rc_chassis->rc.ch[3];
	data_u.data.key_v     = rc_chassis->key.v;
    
    can1_tx_channel_message.StdId = 0x003;
    can1_tx_channel_message.IDE = CAN_ID_STD;
    can1_tx_channel_message.RTR = CAN_RTR_DATA;
    can1_tx_channel_message.DLC = 0x08;
    
    // 发送CAN数据
    HAL_CAN_AddTxMessage(&hcan1, &can1_tx_channel_message, data_u.data_1, &send_mail_box);
}

//摩擦轮控制
void CAN_cmd_shoot(int16_t motor5, int16_t motor6)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = (motor5 >> 8);
    shoot_can_send_data[1] = motor5;
    shoot_can_send_data[2] = (motor6 >> 8);
    shoot_can_send_data[3] = motor6;
    shoot_can_send_data[4] = 0;
    shoot_can_send_data[5] = 0;
    shoot_can_send_data[6] = 0;
    shoot_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

//const can_send_referee_s* get_refree_point()
//{
//		return &gimbal_rx_referee;
//}

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

/**
  * @brief          返回摩擦轮电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,1]
  * @retval         电机数据指针
  */
const motor_measure_t *get_shoot_motor_measure_point(uint8_t i)
{
    return &motor_shoot[(i & 0x03)];
}
