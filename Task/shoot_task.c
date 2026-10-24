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
#include "gimbal_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shot_detection.h"
#include <math.h>

//任务时长常量
#define SHOOT_TASK_INIT_TIME 500
#define SHOOT_CTRL_TIME 5

static TickType_t shootOneStartTime = 0;
#define SHOOT_ONE_DURATION_TICKS pdMS_TO_TICKS(41)  // 单次拨弹持续41 ms
#define AUTO_FIRE_MIN_INTERVAL_TICKS pdMS_TO_TICKS(250) // 自瞄两次拨弹启动至少间隔250 ms
static TickType_t autoFireLastStartTime;
static uint8_t autoFireHasStarted;

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
static shot_detector_t shotDetector;
static shoot_detection_state_t shotDetection;

static void initShootPIDs(void);
static void getInstruction(void);
static void initShootModes(void);
void static triggerModeChange(void);
static void fricModeChange(void);
void setSpeed(void);
static void calcCurrent(void);
static void getShootMotorSpeed(void);
static void tryStartAutoShot(TickType_t now);
static void updateShotDetection(void);

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

        updateShotDetection();           // 只观察摩擦轮掉速，不改变任何电机目标或PID。
		
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
        // 自瞄统一由拨弹状态机读取门控并限频，收帧路径不再直接启动单发。
        fire_t = 0;
	}
}

//摩擦轮状态机
static void fricModeChange(void)
{
	fricOn = 0;

	// 第一步：先按遥控器决定摩擦轮开不开（和原来的逻辑一样）
	if (trigger_state == 1)          // 左拨杆打上：手瞄发弹，开
	{
		fricOn = 1;
	}
	else if (trigger_state == 3)     // 左拨杆打中间：关
	{
		fricOn = 0;
	}
	else if (trigger_state == 2)     // 左拨杆打下：进自瞄，开
	{
		fricOn = 1;
	}

	if (rc_p->mouse.press_r == 1)    // 按住鼠标右键：开
	{
		fricOn = 1;
	}

	// 第二步：在自瞄模式下，如果只需要瞄准、不需要发射，就把摩擦轮关掉
	if (robotIsAuto() == 1)
	{
		// 情况一：正在用串口模拟视觉指令。串口模拟本来就不能开火，所以不开摩擦轮
		if (NX_GetSource() == NX_SOURCE_UART)
		{
			fricOn = 0;
		}

		// 情况二：视觉在 rev 的最低位填了 1，表示这次只瞄准、不发射
		if ((nx_p->rev & 0x0001) == 1)
		{
			fricOn = 0;
		}
	}
}

/* 自瞄单发的唯一入口：从实际启动时刻计间隔，首发无需等待。
 * 冷却期间不排队；开轮、模式切换和重复视觉帧都不重置上次启动时间。
 */
static void tryStartAutoShot(TickType_t now)
{
    if (triggerMode != TriggerMode_e_Stop || !fricOn || !robotIsAuto())
        return;
    if (gimbal_fire_allowed() != 1)
        return;
    if (autoFireHasStarted &&
        (TickType_t)(now - autoFireLastStartTime) < AUTO_FIRE_MIN_INTERVAL_TICKS)
        return;

    triggerMode = TriggerMode_e_ShootOne;
    shootOneStartTime = now;
    autoFireLastStartTime = now;
    autoFireHasStarted = 1;
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

    // 手动连发切入自瞄时也必须经过自瞄的间隔检查。
    if (robotIsAuto() && triggerMode == TriggerMode_e_ShootMulti)
        triggerMode = TriggerMode_e_Stop;
	
    if(lastmode != triggerMode)
    {
        lastmode = triggerMode;
		// 手动单发沿用原计时；自瞄已在统一入口记录实际启动时间。
        if(triggerMode == TriggerMode_e_ShootOne && !robotIsAuto())
        {
            shootOneStartTime = currentTick;
        }
    }
    
    switch(triggerMode)
    {
        case TriggerMode_e_Stop:
            // 鼠标控制发射
            if (rc_p->mouse.press_l == 1 && !robotIsAuto())
            {
                triggerMode = TriggerMode_e_ShootMulti;
            }
            if((robotIsAuto() == 1))
            {
                tryStartAutoShot(currentTick);
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
    autoFireLastStartTime = 0;
    autoFireHasStarted = 0;
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

/* 两轮相向旋转，取原始反馈绝对值的平均；临界区只覆盖读取。 */
static float getFrictionMeanRpm(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    const motor_measure_t *motor0 = get_shoot_motor_measure_point(0);
    const motor_measure_t *motor1 = get_shoot_motor_measure_point(1);
    int16_t speed0 = motor0->speed_rpm;
    int16_t speed1 = motor1->speed_rpm;
    __set_PRIMASK(key);

    float speed_sum_rpm = fabsf((float)speed0);
    speed_sum_rpm += fabsf((float)speed1);
    return speed_sum_rpm * 0.5f;
}

/* 每5 ms观察一次，发布事件时短暂关中断，读者不会拿到一半更新的计数和时刻。 */
static void updateShotDetection(void)
{
    float rpm = getFrictionMeanRpm();
    uint32_t now_ms = HAL_GetTick();
    uint8_t detected = shot_detector_update(&shotDetector, rpm, fricOn, now_ms);

    uint32_t key = __get_PRIMASK();
    __disable_irq();
    // 新事件立即置位；累计数不随关轮清零，供未来视觉识别漏过的短脉冲。
    if (detected)
    {
        shotDetection.count++;
        shotDetection.last_event_ms = now_ms;
        shotDetection.flag = 1U;
    }
    // 只延长显示脉冲，不延迟检测，也不限制下一次真实掉速事件。
    else if ((uint32_t)(now_ms - shotDetection.last_event_ms) >= SHOT_DETECTION_FLAG_MS)
    {
        shotDetection.flag = 0U;
    }
    __set_PRIMASK(key);
}

void shoot_get_detection(shoot_detection_state_t *state)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    *state = shotDetection;
    __set_PRIMASK(key);
}

uint8_t shoot_is_feeding_forward(void)
{
    // 遥控掉线时电控已禁止拨弹输出，不能标记为正在正向拨弹。
    if (!rc_online)
    {
        return 0;
    }
    // 摩擦轮关闭或拨弹未启动时，不把开火请求当作拨弹执行。
    if (!fricOn)
    {
        return 0;
    }
    // 拨弹未启动时，即使其他条件满足也保持未执行状态。
    if (!triggerOn)
    {
        return 0;
    }
    // 只标记正向目标，排除停止和反转解堵；不表示弹丸已经出膛。
    if (wantedVTriggerMotor > 0.0f)
    {
        return 1;
    }
    return 0;
}
