/**
 * @file shoot_task.c
 * @brief 发射控制任务
 * @Auther Jing Yuhang   jyh16213839418@163.com
 * @version 0.2
 * @date 2025-12-21
 *
 *
 * 功能定义：云台摩擦轮和拨弹轮电机控制
 * 支持手动遥控器控制、自动视觉自瞄控制
 * 使用里程碑栈法实现多圈计数，以实现精准发射一颗
 */
#include "cmsis_os.h"
#include "pid.h"
#include "remote_control.h"
#include "NX_communicate.h"
#include "Robot_Information.h"
#include "robot_total_mode.h"
#include "CAN_recieve_send.h"
#include "INS_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

//任务时长常量
#define SHOOT_TASK_INIT_TIME 500
#define SHOOT_CTRL_TIME 5

static TickType_t shootOneStartTime = 0;
#define SHOOT_ONE_DURATION_TICKS pdMS_TO_TICKS(41)  // 200毫秒转换为ticks

#define TRIGGER_ROUNDS_FOR_A_BULLET_A 7
#define TRIGGER_ROUNDS_FOR_A_BULLET_B 8
#define TRIGGER_ROUNDS_FOR_A_BULLET_C 7
#define ECD_FULL_ROUND 8192
#define DEAD_BOND 5

//电机速度环PID
#define M3508_MOTOR_SPEED_PID_KP 60000.0f
#define M3508_MOTOR_SPEED_PID_KI 0.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f

#define M2006_MOTOR_SPEED_PID_KP 20000.0f
#define M2006_MOTOR_SPEED_PID_KI 0.0f   //不要这个积分项，否则连续发射停止后还会转
#define M2006_MOTOR_SPEED_PID_KD 1000.0f

#define MAX_MOTOR_3508_CURRENT 30000.0f
#define MAX_MOTOR_2006_CURRENT 30000.0f

#define M3505_MOTOR_SHOOT_SPEED_PID_MAX_OUT    MAX_MOTOR_3508_CURRENT  // PID总输出最大值
#define M3505_MOTOR_SHOOT_SPEED_PID_MAX_IOUT   2000.0f                 // PID积分项最大值

#define M2006_MOTOR_SPEED_PID_MAX_OUT          MAX_MOTOR_2006_CURRENT  // PID总输出最大值
#define M2006_MOTOR_SPEED_PID_MAX_IOUT         2000.0f                 // PID积分项最大值

#define SHOOT_M3508_MOTOR_RPM_TO_VECTOR (0.000415809748903494517209f/5)
#define SHOOT_M2006_MOTOR_RPM_TO_VECTOR (0.000415809748903494517209f/5)

const static fp32 fric_motor_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
const static fp32 trigger_motor_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};

//拨弹轮电机模式枚举类型
enum TriggerMode_e{
    TriggerMode_e_Stop,         //停止发射
    TriggerMode_e_ShootOne,     //发射一个，靠拨弹轮减速前的电机转4圈。对应弹盘轮子转一格
    TriggerMode_e_ShootMulti,   //多发，其实是等待进入发射一个的状态
	TriggerMode_e_StuckSolve,
};

static const CAN_Send_to_MCU *nx_p;
static const RC_ctrl_t *rc_p;
static int16_t trigger_state=3;//拨杆状态
static int16_t wheel=0;		   //拨轮值

// 摩擦轮开启
bool_t fricOn=0;
//拨弹轮开启
uint8_t triggerOn=0;

extern volatile uint8_t  rc_online;
extern volatile uint8_t data_ready;

static enum TriggerMode_e triggerMode;

static int16_t giveShootCurrent[2];
static volatile int16_t giveTriggerCurrent;

static fp32 wantedVShootMotor[2],wantedVTriggerMotor;
static fp32 presentVShootMotor[2],presentVTriggerMotor;

static pid_type_def shootMotorPIDs[2],triggerMotorPID;

static void initShootPIDs(void);
static void getInstruction(void);
static void initShootModes(void);
void static triggerModeChange(void);
static void fricModeChange(void);
void setSpeed(void);
static void calcCurrent(void);
static void getShootMotorSpeed(void);

void shoot_task(void const *pvParameters)
{
    osDelay(SHOOT_TASK_INIT_TIME);
	
    initShootPIDs();    //初始化PID
    initShootModes();   //初始化发射状态

    rc_p = get_remote_control_point();    //获取遥控器数据和NNX数据指针
	nx_p = get_nx_control_point();

    while(1)
    {
        getInstruction();
		
		fricModeChange();           //摩擦轮状态改变

        triggerModeChange();        //拨弹轮状态转变
		
        setSpeed();
		
        getShootMotorSpeed();
		
        calcCurrent();                   //计算PID

        CAN_cmd_shoot(giveShootCurrent[0], giveShootCurrent[1]);

		osDelay(SHOOT_CTRL_TIME);
    }
}

static void initShootPIDs(void)
{
    uint8_t i;
    for(i=0;i<2;i++)
        PID_init(&shootMotorPIDs[i],PID_POSITION,fric_motor_speed_pid,M3505_MOTOR_SHOOT_SPEED_PID_MAX_OUT,M3505_MOTOR_SHOOT_SPEED_PID_MAX_IOUT);
	
    PID_init(&triggerMotorPID,PID_POSITION,trigger_motor_speed_pid,M2006_MOTOR_SPEED_PID_MAX_OUT,M2006_MOTOR_SPEED_PID_MAX_IOUT);
}
uint8_t fire_t;
//添加NX控制发射
static void getInstruction(void)
{
	trigger_state=rc_p->rc.s[1];//拨杆，摩擦轮
	wheel		 =rc_p->rc.ch[4];
	//拨轮大于某值连发，小于某值单发
	//左拨杆 上：开摩擦轮 中：关 下：nx接管，并且改变总状态机以改变robotauto
	if(robotIsAuto())//自动模式
	{
		if(fire_t == 1)
		{
			triggerMode = TriggerMode_e_ShootOne;
			fire_t = 0;//凌晨写的逻辑，我觉得有问题
		}
	}
}

//摩擦轮状态机
static void fricModeChange(void)
{
	fricOn=0;
	if(trigger_state==1 || rc_p->mouse.press_r==1)//开
	{
		fricOn=1;
	}
	else if(trigger_state==3)//关
	{
		fricOn=0;
	}
	else if(trigger_state==2)//自动
	{
		fricOn=1;//进自动模式全程开摩擦轮
	}
}

//拨弹轮状态机
void static triggerModeChange(void)
{
    static uint8_t shootOneLatched = 0;
	static uint8_t lastmode = TriggerMode_e_Stop;
    static TickType_t currentTick = 0;
    if(!fricOn) 
    {
        triggerMode = TriggerMode_e_Stop;
        return;
    }
	
    currentTick = xTaskGetTickCount();
	
    if(lastmode != triggerMode)
    {
        lastmode = triggerMode;
		// 进入单发模式时记录开始时间
        if(triggerMode == TriggerMode_e_ShootOne)
        {
            shootOneStartTime = currentTick;
        }
    }
    
    switch(triggerMode)
    {
        case TriggerMode_e_Stop:
            // 鼠标控制发射
            if (rc_p->mouse.press_l == 1)
            {
                triggerMode = TriggerMode_e_ShootMulti;
            }
            if((robotIsAuto() == 1))
            {
                if(nx_p->fire == 1)
                {
                    triggerMode = TriggerMode_e_ShootOne;
                }
            }
            else //手动模式手柄
            {
                if(wheel >= 50 && wheel <= 400)
                {
                    if(!shootOneLatched)
                    {
                        triggerMode = TriggerMode_e_ShootOne;
                        shootOneLatched = 1;
                    }
                }
                else if(wheel > 400)  // 摇杆在连发区间
                {
                    triggerMode = TriggerMode_e_ShootMulti;
                }
                else if(wheel < -50)
                {
                    triggerMode = TriggerMode_e_StuckSolve;
                }
                else
                {
                    shootOneLatched = 0; // 退回死区才允许下一次单发
                }
            }
            break;
            
        case TriggerMode_e_ShootOne:
            if((currentTick - shootOneStartTime) >= SHOOT_ONE_DURATION_TICKS)
            {
                triggerMode = TriggerMode_e_Stop;
            }
            break;
            
        case TriggerMode_e_ShootMulti:
            // 连发状态下，摇杆离开连发区间就停止
            if(wheel <= 400)  // 摇杆离开连发区间
            {
                triggerMode = TriggerMode_e_Stop;
            }
            // 否则保持连发状态
            break;
            
        case TriggerMode_e_StuckSolve:
            // 卡弹解决状态
            if(wheel>=-50)
			{
				triggerMode = TriggerMode_e_Stop;
			}
            // 解决完成后回到停止状态
            break;
            
        default:
            triggerMode = TriggerMode_e_Stop;
            break;
    }
}

static void initShootModes(void)
{
    //初始化指令
    fricOn=0;
    triggerOn=0;
	triggerMode=TriggerMode_e_Stop;
}

//速度设置
void setSpeed(void)
{
	if(fricOn)
	{
		wantedVShootMotor[0] = 5;
		wantedVShootMotor[1] = -5;
	}
	else
	{
		wantedVShootMotor[0] = 0;
		wantedVShootMotor[1] = 0;
	}
	
	if(triggerMode==TriggerMode_e_ShootOne || triggerMode==TriggerMode_e_ShootMulti)
	{
		triggerOn=1;
	}
	else if(triggerMode==TriggerMode_e_StuckSolve)
	{
		triggerOn=1;
	}
	else
	{
		triggerOn=0;
	}
	
	if(triggerOn)
	{
		if(triggerMode==TriggerMode_e_StuckSolve)
		{
			wantedVTriggerMotor = -0.525;
		}
		else
		{
			wantedVTriggerMotor = 0.525;
		}
	}
	else
	{
		wantedVTriggerMotor = 0;
	}
}

static void calcCurrent(void)
{
	for(int i=0;i<2;i++)
	{
		PID_calc(&shootMotorPIDs[i],presentVShootMotor[i],wantedVShootMotor[i]);
	}
	
	for(int i=0;i<2;i++)
        giveShootCurrent[i]=shootMotorPIDs[i].out;
	
	PID_calc(&triggerMotorPID,presentVTriggerMotor,wantedVTriggerMotor);
    giveTriggerCurrent=triggerMotorPID.out;
	
	if (!rc_online)
	{
		giveTriggerCurrent=0;
	}
}

static void getShootMotorSpeed(void)
{
    for(int i=0;i<2;i++)
    {
        presentVShootMotor[i]=get_shoot_motor_measure_point(i)->speed_rpm*SHOOT_M3508_MOTOR_RPM_TO_VECTOR;
    }	
    presentVTriggerMotor=get_trigger_motor_measure_point()->speed_rpm*SHOOT_M2006_MOTOR_RPM_TO_VECTOR;
}

volatile int16_t * getTriggerCurrentP(void)
{
    return &giveTriggerCurrent;
}

bool_t get_fric(void)
{
	return fricOn;
}
