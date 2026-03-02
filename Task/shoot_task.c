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

//任务时长常量
#define SHOOT_TASK_INIT_TIME 500
#define SHOOT_CTRL_TIME 5

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

#define MILESTONE_NEAR_THRESHHOLD 150		//里程碑栈法参数
#define MILESTONE_NUMBER 3
struct milestoneStack_s {
    uint8_t head;
    uint8_t stack[MILESTONE_NUMBER+1];
};
struct TriggerControl_s{
    struct milestoneStack_s mstack; 	//里程碑栈
    const uint16_t * ECDPoint;          //电机ECD所在位置
    uint16_t    initECD;                //初始ECD
    uint16_t    nowECD;                 //现在的ECD
    int16_t     nowRounds;              //现在转过的圈数
};

const static fp32 fric_motor_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
const static fp32 trigger_motor_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};

//拨弹轮电机模式枚举类型
enum TriggerMode_e{
    TriggerMode_e_Stop,         //停止发射
    TriggerMode_e_ShootOne,     //发射一个，靠拨弹轮减速前的电机转4圈。对应弹盘轮子转一格
    TriggerMode_e_ShootMulti,   //多发，其实是等待进入发射一个的状态
	TriggerMode_e_StuckSolve,
};

static const VisionToGimbal_t *nx_p;
static const RC_ctrl_t *rc_p;
static int16_t trigger_state=3;//拨杆状态
static int16_t wheel=0;		   //拨轮值

// 摩擦轮开启
bool_t fricOn=0;
//拨弹轮开启
uint8_t triggerOn=0;

extern volatile uint8_t  rc_online;
extern volatile uint8_t data_ready;

static struct TriggerControl_s triggerCtrl;
static enum TriggerMode_e triggerMode;

static int16_t giveShootCurrent[2];
static volatile int16_t giveTriggerCurrent;

static fp32 wantedVShootMotor[2],wantedVTriggerMotor;
static fp32 presentVShootMotor[2],presentVTriggerMotor;

static pid_type_def shootMotorPIDs[2],triggerMotorPID;

static void initShootPIDs(void);
static void initTriggerECDRoundsMonitor(void);
static void getInstruction(void);
static void monitorTriggerECDRound(void);
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
    initTriggerECDRoundsMonitor();  //初始化拨弹轮圈数监控
    initShootModes();   //初始化发射状态

    rc_p = get_remote_control_point();    //获取遥控器数据和NNX数据指针
	nx_p = get_nx_control_point();

    while(1)
    {
        getInstruction();
		fricModeChange();           //摩擦轮状态改变

        triggerModeChange();        //拨弹轮状态转变
		
        setSpeed();

        monitorTriggerECDRound();        //监控拨弹轮ECD圈数
		
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

/**
 * @brief 将ECD差值等化为(0,8191)范围
 * 
 * @param rawECD 
 * @return uint16_t 
 */
static uint16_t ECDFormat(int16_t rawECD)     //test done
{
    while(rawECD<0)
        rawECD+=ECD_FULL_ROUND;
    while(rawECD>=ECD_FULL_ROUND)
        rawECD-=ECD_FULL_ROUND;
    return (uint16_t)rawECD;
}

static void initTriggerECDRoundsMonitor(void)  //初始化拨弹轮圈数监控
{
    triggerCtrl.ECDPoint=&(get_trigger_motor_measure_point()->ecd);
    triggerCtrl.initECD=*(triggerCtrl.ECDPoint);
	
	triggerCtrl.nowRounds = 0;
	triggerCtrl.mstack.head = 0;
	triggerCtrl.mstack.stack[0] = 0;
}

//添加NX控制发射
static void getInstruction(void)
{
    if(robotIsAuto())//自动模式
	{
		if((nx_p->mode == 2) && data_ready)
		{
			triggerMode = TriggerMode_e_ShootOne;
		}
		return;
	}
	else
	{
		trigger_state=rc_p->rc.s[1];//拨杆，摩擦轮
		wheel		 =rc_p->rc.ch[4];
		//拨轮大于某值连发，小于某值单发
		//左拨杆 上：开摩擦轮 中：关 下：nuc接管，并且改变总状态机以改变robotauto
		
	}
}

/**
 * @brief 监控拨弹轮电机ECD并更新圈数计数
 * 使用里程碑栈法检测电机转动的完整圈数
 * 将一圈8192个ECD值划分为MILESTONE_NUMBER个区域（里程碑）
 * 通过检测里程碑出现的顺序和模式来判断是否完成一圈
 */
static void monitorTriggerECDRound(void)
{
    uint8_t j;
    // 获取当前电机的ECD值（0-8191）
    triggerCtrl.nowECD = *(triggerCtrl.ECDPoint);

    // 遍历所有里程碑区域，检查当前ECD是否在某个里程碑附近
    for(j = 0; j < MILESTONE_NUMBER; j++)
    {
        // 计算相对于初始位置的ECD值（处理溢出）
        fp32 relativeRealECD = ECDFormat((int16_t)triggerCtrl.nowECD - (int16_t)triggerCtrl.initECD);
        
        // 计算当前ECD与第j个里程碑理论位置的距离
        // 第j个里程碑的理论位置 = j * ECD_FULL_ROUND / MILESTONE_NUMBER
        fp32 distance_to_milestone;
        int32_t raw_distance = (int32_t)relativeRealECD - (j * ECD_FULL_ROUND / MILESTONE_NUMBER);
        
        // 处理距离的模运算，找到最短距离（考虑ECD循环特性）
        if (raw_distance > ECD_FULL_ROUND/2) {
            distance_to_milestone = ECDFormat((int16_t)(raw_distance - ECD_FULL_ROUND));
        } else if (raw_distance < -ECD_FULL_ROUND/2) {
            distance_to_milestone = ECDFormat((int16_t)(raw_distance + ECD_FULL_ROUND));
        } else {
            distance_to_milestone = ECDFormat((int16_t)raw_distance);
        }
        
        // 检查是否接近里程碑（在阈值范围内）
        if (distance_to_milestone < MILESTONE_NEAR_THRESHHOLD || 
            (ECD_FULL_ROUND - distance_to_milestone) < MILESTONE_NEAR_THRESHHOLD)
        {
            // 当前位置落在里程碑j的区域内
            if (j != triggerCtrl.mstack.stack[triggerCtrl.mstack.head])
            {
                // 到达一个新的里程碑，将其压入栈中
                if (triggerCtrl.mstack.head < MILESTONE_NUMBER)
                {
                    triggerCtrl.mstack.head++;
                    triggerCtrl.mstack.stack[triggerCtrl.mstack.head] = j;
                }
                else
                {
                    // 栈满处理（这种情况理论上不应该发生）
                    // 清空栈从头开始
                    triggerCtrl.mstack.head = 0;
                    triggerCtrl.mstack.stack[0] = j;
                }
            }
            // 找到一个里程碑就可以退出循环了
            break;
        }
    }
    
    // 检查栈中是否有足够的元素来进行模式匹配
    if (triggerCtrl.mstack.head >= 3)
    {
        // 模式1：检查是否出现0->1->2->0或1->2->0->1或2->0->1->2等正向模式
        // 对于MILESTONE_NUMBER=3的情况，检测三个连续的递增序列（模3）
        int8_t diff1 = (triggerCtrl.mstack.stack[triggerCtrl.mstack.head] - 
                       triggerCtrl.mstack.stack[triggerCtrl.mstack.head-1] + MILESTONE_NUMBER) % MILESTONE_NUMBER;
        int8_t diff2 = (triggerCtrl.mstack.stack[triggerCtrl.mstack.head-1] - 
                       triggerCtrl.mstack.stack[triggerCtrl.mstack.head-2] + MILESTONE_NUMBER) % MILESTONE_NUMBER;
        int8_t diff3 = (triggerCtrl.mstack.stack[triggerCtrl.mstack.head-2] - 
                       triggerCtrl.mstack.stack[triggerCtrl.mstack.head-3] + MILESTONE_NUMBER) % MILESTONE_NUMBER;
        
        // 正向旋转：三个差值都应为1（模3）
        if (diff1 == 1 && diff2 == 1 && diff3 == 1)
        {
            triggerCtrl.nowRounds += 1;  // 完成正向一圈
            // 弹出前三个元素，保留最后一个作为下一圈的起点
            triggerCtrl.mstack.stack[0] = triggerCtrl.mstack.stack[triggerCtrl.mstack.head];
            triggerCtrl.mstack.head = 0;
        }
        // 反向旋转：三个差值都应为2（即-1模3）
        else if (diff1 == MILESTONE_NUMBER-1 && diff2 == MILESTONE_NUMBER-1 && diff3 == MILESTONE_NUMBER-1)
        {
            triggerCtrl.nowRounds -= 1;  // 完成反向一圈
            // 弹出前三个元素，保留最后一个作为下一圈的起点
            triggerCtrl.mstack.stack[0] = triggerCtrl.mstack.stack[triggerCtrl.mstack.head];
            triggerCtrl.mstack.head = 0;
        }
    }
}

static void initShootModes(void)
{
    //初始化指令
    fricOn=0;
    triggerOn=0;
	triggerMode=TriggerMode_e_Stop;
}



//拨弹轮状态机
void static triggerModeChange(void)
{
    static uint8_t lastmode = TriggerMode_e_Stop;
    static uint8_t nowTimeRoundThreshold = TRIGGER_ROUNDS_FOR_A_BULLET_A;
    
	static uint8_t shootOneLatched = 0;
    // 摩擦轮关闭状态，始终让拨弹轮关闭
    if(!fricOn) 
    {
        triggerMode = TriggerMode_e_Stop;
//        return;
    }
    
    // 记录上次状态用于状态变化检测
    if(lastmode != triggerMode)
    {
        lastmode = triggerMode;
    }
    switch(triggerMode)
    {
        case TriggerMode_e_Stop:
			// 鼠标控制发射
			if (rc_p->mouse.press_l == 1)
			{
				triggerMode = TriggerMode_e_ShootMulti;
			}
            // 停止状态下，可以根据摇杆位置进入单发或连发
            if(wheel >= 50 && wheel <= 400)
			{
				if(!shootOneLatched)
				{
					triggerMode = TriggerMode_e_ShootOne;
					shootOneLatched = 1;
					nowTimeRoundThreshold = TRIGGER_ROUNDS_FOR_A_BULLET_A;
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
            // 摇杆在死区或小于50，保持停止状态
            break;
            
        case TriggerMode_e_ShootOne:
            // 单发状态下，必须等待单发完成才能切换状态
            // 检查是否达到预定圈数（正转或反转4圈）

			//JYH：没有shootonelatch标志位会反复进入单发状态导致伪连发
            if(triggerCtrl.nowRounds <= (-nowTimeRoundThreshold) || 
               triggerCtrl.nowRounds >= nowTimeRoundThreshold)
            {
                // 单发完成，切换回停止状态
                triggerMode = TriggerMode_e_Stop;
                triggerCtrl.nowRounds = 0;  // 重置圈数
                triggerCtrl.mstack.head = 0;
				triggerCtrl.mstack.stack[0] = 0;
                // 轮换圈数阈值，避免机械共振
                if(nowTimeRoundThreshold == TRIGGER_ROUNDS_FOR_A_BULLET_A)
                    nowTimeRoundThreshold = TRIGGER_ROUNDS_FOR_A_BULLET_B;
                else if(nowTimeRoundThreshold == TRIGGER_ROUNDS_FOR_A_BULLET_B)
                    nowTimeRoundThreshold = TRIGGER_ROUNDS_FOR_A_BULLET_C;
                else if(nowTimeRoundThreshold == TRIGGER_ROUNDS_FOR_A_BULLET_C)
                    nowTimeRoundThreshold = TRIGGER_ROUNDS_FOR_A_BULLET_A;
            }
            // 否则保持单发状态（忽略摇杆变化）
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
	else if(trigger_state==2 || robotIsAuto())//自动
	{
		fricOn=1;//进自动模式全程开摩擦轮
	}
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
