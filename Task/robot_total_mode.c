/**
 * @file robot_total_mode.c
 * @brief 最顶层状态机，决定全车模式
 * @Auther Jing Yuhang  jyh16213839418@163.com
 * @version 0.2
 * @date 2025-12-21
 *
 */
#include "cmsis_os.h"
#include "robot_total_mode.h"
#include "Robot_Information.h"
#include "remote_control.h"
//#include "nucCommu.h"
#include "bsp_buzzer.h"
#include "main.h"

#define ROBOT_MODE_TASK_TIME 10	
#define RC_ROBOT_MODE_CHANNEL 0

static int robotAuto=0;     //自动/手动模式标志，手动0自动1
static const RC_ctrl_t *rc_p;
//static const toSTM32_t *nuc_p;  //NUC数据位置 

static enum RobotState_e robotState=RobotState_e_Powerless;  //机器人初始状态
static uint8_t controlmode = 0;		//0：手柄   1：键盘
extern volatile uint8_t  rc_online;

/*当键盘有任何按键开始按下后，就转换到键盘和鼠标控制。
 *当遥控器拨轮发生状态改变，遥控器控制
 *
 */ 

void robot_total_mode_task(void const *pvParameters)
{
    //初始化数据指针
    rc_p=get_remote_control_point();
//    nuc_p=get_nuc_control_point();

    robotAuto=0;
    while (1)
    {
        // 遥控器掉线时，强制回到PowerLess状态
        // 当键盘有任何按键开始按下后，就转换到键盘和鼠标控制了
		
		CtrlModeDetect();

        if(rc_online == 0)
        {
            robotState = RobotState_e_Powerless;
            osDelay(ROBOT_MODE_TASK_TIME);
            continue;
        }
		
		if(controlmode==0)//手柄
        {
            if(switch_is_mid(rc_p->rc.s[RC_ROBOT_MODE_CHANNEL]))
            {
                robotState=RobotState_e_GimbalCar;//发送3,底盘跟随云台
            }
            else if(switch_is_down(rc_p->rc.s[RC_ROBOT_MODE_CHANNEL]))
            {
                robotState=RobotState_e_Spinner;//发送2，小陀螺
            }
            else if(switch_is_up(rc_p->rc.s[RC_ROBOT_MODE_CHANNEL]))
            {
                //未定义功能
            }
        }
		else//键盘
		{
			if(rc_p->key.v & KEY_PRESSED_OFFSET_C)   //c开启云台车模式
			{
				robotState=RobotState_e_GimbalCar;
			}
			
			if(rc_p->key.v & KEY_PRESSED_OFFSET_CTRL)   //ctrl开启小陀螺
			{
				robotState=RobotState_e_Spinner;
			}
		}
		
		if(rc_p->rc.s[0]==2)
        {
            robotAuto=1;
        }
        else
        {
            robotAuto=0;
        }
		
		osDelay(ROBOT_MODE_TASK_TIME);
	}
}

void CtrlModeDetect(void)
{
	if(rc_p->rc.ch[4]>=20||rc_p->rc.ch[4]<=-20)
	{
		controlmode = 0;
	}
	
	if(rc_p->key.v != 0)
	{
		controlmode = 1;
	}
}

int robotIsAuto(void)
{
    return robotAuto;
}

const enum RobotState_e * getRobotPresentMode(void)
{
    return &robotState;
}
