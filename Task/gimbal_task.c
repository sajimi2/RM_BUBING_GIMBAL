/**
 * @file gimbal_task.c
 * @brief 云台控制任务
 * @Auther Jing Yuhang  jyh16213839418@163.com
 * @version 0.3
 * @date 2025-12-21
 *
 *
 * 功能定义：云台电机控制，包含Pitch轴和Yaw轴的串级PID控制，
 * 支持手动遥控器控制、鼠标控制和自动视觉自瞄控制。
 * 使用里程碑栈法实现多圈计数，确保角度连续。
 */

#include "cmsis_os.h"
#include "main.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_recieve_send.h"
#include "user_lib.h"
#include "struct_typedef.h"
#include "INS_task.h"
#include "gimbal_task.h"
#include "math.h"
#include "Robot_Information.h"
#include "robot_total_mode.h"
#include "bsp_laser.h"
#include "shoot_task.h"
#include "NX_communicate.h"

// 任务周期
#define GIMBAL_TASK_CTRL_TIME 2

// 速度环PID参数
#define PITCH_SPD_KP 4000000.0f
#define PITCH_SPD_KI 000.0f
#define PITCH_SPD_KD 1000.0f

#define YAW_SPD_KP 10000000.0f
#define YAW_SPD_KI 0	
#define YAW_SPD_KD 0

// 位置环PID参数
#define PITCH_AGL_KP 0.015f
#define PITCH_AGL_KI 0
#define PITCH_AGL_KD 0.01f

#define YAW_AGL_KP 0.006f
#define YAW_AGL_KI 0.00f
#define YAW_AGL_KD 0.00000f

// 功率保护参数
#define PITCH_VOLT_MAX_OUT 30000.0f
#define PITCH_VOLT_MAX_IOUT 3000.0f
#define YAW_VOLT_MAX_OUT 30000.0f
#define YAW_VOLT_MAX_IOUT 10000.0f

// 位置环输出限幅
#define YAW_AGL_SPD_MAX_OUT (50.0f)
#define YAW_AGL_SPD_MAX_IOUT (11.7f)
#define PITCH_AGL_SPD_MAX_OUT (2.0f)
#define PITCH_AGL_SPD_MAX_IOUT (1.0f)

// 摇杆操作累计量参数
#define YAW_RC_SEN -0.000004f
#define PITCH_RC_SEN -0.000005f
#define YAW_MOUSE_SEN -0.000005f
#define PITCH_MOUSE_SEN -0.000005f

// 编码器相关
#define ECD_FULL_ROUND 8192

// 死区参数
#define RC_DEADLINE 10
#define GIMBAL_DEAD_BOND 0.1
#define YAW_DEAD_BOND 0.1

// Pitch轴角度限幅
#define PITCH_MAX_ANGLE 0.7f  // 0.275763俯角
#define PITCH_MIN_ANGLE -0.45f //-0.280368仰角0.22

// 遥控器死区宏函数
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

// 限幅宏函数
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#define range_limit_inside(toBeLimited, range) \
    {                                          \
        if ((toBeLimited) > (range))           \
        {                                      \
            (toBeLimited) = (range);           \
        }                                      \
        else if ((toBeLimited) < -(range))     \
        {                                      \
            (toBeLimited) = -(range);          \
        }                                      \
    }

// 云台电机控制结构体
struct GimbalMotorCtrl_s
{
    // 角度相关
    const fp32 *AnglePoint;
    fp32 nowAbsoluteAngle;
    fp32 lastAbsoluteAngle;
    uint32_t nowTime;
    uint32_t lastTime;
    fp32 radspeed;
    fp32 want_AbsoluteAngleCtrl;

    // 编码器相关
    const uint16_t *ECDPoint;
    uint16_t maxECD, minECD, middleECD, nowECD;

    // 里程碑相关
    int16_t nowRounds;
    int16_t maxRounds;

    // PID相关
    pid_type_def spd_pid;
    pid_type_def agl_pid;

    // 控制输出
    int16_t Volt_out;
    uint8_t nopowerflag;

    // 滤波器
    first_order_filter_type_t spd_filter;
};

// 云台控制结构体
static struct GimbalMotorCtrl_s PitchCtrl, YawCtrl;
static struct GimbalMotorCtrl_s *gimbalCtrl[2] = {&PitchCtrl, &YawCtrl};

// PID参数数组
static fp32 pitchSpdPIDco[3] = {PITCH_SPD_KP, PITCH_SPD_KI, PITCH_SPD_KD};
static fp32 yawSpdPIDco[3] = {YAW_SPD_KP, YAW_SPD_KI, YAW_SPD_KD};
static fp32 pitchAglPIDco[3] = {PITCH_AGL_KP, PITCH_AGL_KI, PITCH_AGL_KD};
static fp32 yawAglPIDco[3] = {YAW_AGL_KP, YAW_AGL_KI, YAW_AGL_KD};

// 模式相关
static const enum RobotState_e *robotMode;
static enum RobotState_e lastMode;

// 控制指针
static const RC_ctrl_t *rc_p;
static const CAN_Send_to_MCU *nx_p;

extern volatile uint8_t rc_online;
extern int16_t *getTriggerCurrentP(void);

// 函数声明
void gimbal_task(void const *pvParameters);
const struct GimbalMotorCtrl_s *get_wantedyaw_point(void);
static void initGimbalCtrl(void);
void refreshAngleStates(struct GimbalMotorCtrl_s *c);
void getControlAngles(void);
void unitPID_calc(void);

fp32 delta_pitch, delta_yaw;
/**
 * @brief 云台主控制任务
 */
void gimbal_task(void const *pvParameters)
{
    uint8_t i;

    // 初始化延迟
    osDelay(500);

    // 初始化云台控制
    initGimbalCtrl();
    robotMode = getRobotPresentMode();
    rc_p = get_remote_control_point(); // 获取遥控器数据指针
    nx_p = get_nx_control_point();     // 视觉数据指针

    delta_pitch = nx_p->pitch;
    delta_yaw = nx_p->yaw;

    // 初始化延迟
    osDelay(500);

    // 清除PID积分
    PID_clear(&(YawCtrl.spd_pid));
    PID_clear(&(YawCtrl.agl_pid));
    PID_clear(&(PitchCtrl.spd_pid));
    PID_clear(&(PitchCtrl.agl_pid));

    // 初始化状态
    lastMode = RobotState_e_Powerless; // 上一次初始化为无力
    YawCtrl.nopowerflag = 0;
    PitchCtrl.nopowerflag = 0;
    YawCtrl.want_AbsoluteAngleCtrl = YawCtrl.nowAbsoluteAngle;
    PitchCtrl.want_AbsoluteAngleCtrl = PitchCtrl.nowAbsoluteAngle;
    // 获取拨弹轮电流指针
    int16_t *triggerCurrentP = getTriggerCurrentP(); // 从另一个文件获取拨弹轮电机的电流

    // 主控制循环
    while (1)
    {
        // 刷新角度状态
        for (i = 0; i < 2; i++)
            refreshAngleStates(gimbalCtrl[i]);

        // 获取控制角度
        getControlAngles();

        // 计算PID
        unitPID_calc();

        // 遥控器在线侦测
		// LYL
        if (rc_online == 0)
        {
            YawCtrl.Volt_out = 0;
            PitchCtrl.Volt_out = 0;
        }
		else
		{
		}

        // 发送控制指令到CAN总线
        CAN_cmd_gimbal(YawCtrl.Volt_out, PitchCtrl.Volt_out, *triggerCurrentP, 0);

        // 任务延时
        osDelay(GIMBAL_TASK_CTRL_TIME);
    }
}

/**
 * @brief 初始化云台控制结构体
 */
#define PITCH_SPD_FILTER_NUM 0.001f
#define YAW_SPD_FILTER_NUM 0.001f
static void initGimbalCtrl(void)
{
    // 初始化角度指针
    YawCtrl.AnglePoint = &(get_INS()->Yaw);
    PitchCtrl.AnglePoint = &(get_INS()->Pitch);

    // 初始化编码器指针
    YawCtrl.ECDPoint = &(get_yaw_gimbal_motor_measure_point()->ecd);
    PitchCtrl.ECDPoint = &(get_pitch_gimbal_motor_measure_point()->ecd);

    // PID初始化
    PID_init(&(YawCtrl.spd_pid), PID_POSITION, yawSpdPIDco, YAW_VOLT_MAX_OUT, YAW_VOLT_MAX_IOUT);
    PID_init(&(PitchCtrl.spd_pid), PID_POSITION, pitchSpdPIDco, PITCH_VOLT_MAX_OUT, PITCH_VOLT_MAX_IOUT);

    PID_init(&(PitchCtrl.agl_pid), PID_POSITION, pitchAglPIDco, PITCH_AGL_SPD_MAX_OUT, PITCH_AGL_SPD_MAX_IOUT);
    PID_init(&(YawCtrl.agl_pid), PID_POSITION, yawAglPIDco, YAW_AGL_SPD_MAX_OUT, YAW_AGL_SPD_MAX_IOUT);

    // 滤波初始化
    const static fp32 gimbal_pitch_order_filter[1] = {PITCH_SPD_FILTER_NUM};
    const static fp32 gimbal_yaw_order_filter[1] = {YAW_SPD_FILTER_NUM};

    first_order_filter_init(&(PitchCtrl.spd_filter), GIMBAL_TASK_CTRL_TIME, gimbal_pitch_order_filter);
    first_order_filter_init(&(YawCtrl.spd_filter), GIMBAL_TASK_CTRL_TIME, gimbal_yaw_order_filter);

    // 初始化当前状态
    for (uint8_t i = 0; i < 2; i++)
    {
        (gimbalCtrl[i])->nopowerflag = 0;
        (gimbalCtrl[i])->nowAbsoluteAngle = *((gimbalCtrl[i])->AnglePoint);
        (gimbalCtrl[i])->nowTime = HAL_GetTick();
        gimbalCtrl[i]->want_AbsoluteAngleCtrl = (gimbalCtrl[i])->nowAbsoluteAngle;
    }
}

/**
 * @brief 将角度化为(-PI,PI)范围内
 *
 * @param rawAngle
 * @return fp32
 */
fp32 radFormat(fp32 rawAngle) // test done
{
    while (rawAngle > PI)
        rawAngle -= (2 * PI);
    while (rawAngle < (-PI))
        rawAngle += (2 * PI);
    return rawAngle;
}

/**
 * @brief 将ECD差值等化为(0,8191)范围
 *
 * @param rawECD
 * @return uint16_t
 */
static uint16_t ECDFormat(int16_t rawECD) // test done
{
    while (rawECD < 0)
        rawECD += ECD_FULL_ROUND;
    while (rawECD >= ECD_FULL_ROUND)
        rawECD -= ECD_FULL_ROUND;
    return (uint16_t)rawECD;
}

/**
 * @brief 将0-8191的ECD值转化为-PI~PI的弧度值,只是映射一一过去
 * 0->-PI,8192->PI
 *
 * @param ecd
 * @return fp32
 */
fp32 ECD2Rad(uint16_t ecd) // test done
{
    return ((fp32)ecd) * (2 * PI) / ECD_FULL_ROUND - PI;
}

/**
 * @brief 计算转圈的PID。内部error先循环转化为(-PI,PI)范围
 *
 * @param pid
 * @param ref 反馈值
 * @param set 设定值
 * @return fp32
 */
fp32 gimbal_PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = radFormat(set - ref);
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        range_limit_inside(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        range_limit_inside(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        if (pid->error[0] < 100)
        {
            pid->error[0] = 0;
        }
        else
        {
            pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
            pid->Iout = pid->Ki * pid->error[0];
            pid->Dbuf[2] = pid->Dbuf[1];
            pid->Dbuf[1] = pid->Dbuf[0];
            pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
            pid->Dout = pid->Kd * pid->Dbuf[0];
            pid->out += pid->Pout + pid->Iout + pid->Dout;
        }
    }
    return pid->out;
}

/**
 * @brief 刷新角度状态（计算当前角度和角速度）
 */
void refreshAngleStates(struct GimbalMotorCtrl_s *c)
{
    c->lastAbsoluteAngle = c->nowAbsoluteAngle;
    c->lastTime = c->nowTime;

    c->nowAbsoluteAngle = *(c->AnglePoint);
    c->nowTime = xTaskGetTickCount();

    uint32_t dt = c->nowTime - c->lastTime; // dt单位是ms
    if (dt == 0)
    {
        c->radspeed = 0.0f;
        c->nowECD = *(c->ECDPoint);
        return;
    }

    float angleDifference = c->nowAbsoluteAngle - c->lastAbsoluteAngle;

    if (angleDifference > PI)
        angleDifference -= 2 * PI;
    else if (angleDifference < -PI)
        angleDifference += 2 * PI;

    c->radspeed = angleDifference / dt;
    c->nowECD = *(c->ECDPoint);
}

/**
 * @brief 获取云台控制角度（状态机）
 *
 * 状态机包含三种主要状态：
 * 1. 无力模式（RobotState_e_Powerless）
 * 2. 手动控制模式（非自动模式）
 * 3. 自动控制模式（robotIsAuto()返回1）
 *
 * 注意：从无力模式切换到其他模式时，需要初始化目标角度为当前角度
 */
void getControlAngles(void)
{
    // 获取遥控器输入（带死区处理）
    int16_t yaw_channel = 0, pitch_channel = 0;
    rc_deadband_limit(rc_p->rc.ch[0], yaw_channel, RC_DEADLINE);   // 右摇杆X轴
    rc_deadband_limit(rc_p->rc.ch[1], pitch_channel, RC_DEADLINE); // 右摇杆Y轴

	// 清除无力标志
	// LYL
    YawCtrl.nopowerflag = 0;
    PitchCtrl.nopowerflag = 0;
	
    if (*robotMode == RobotState_e_Powerless)
    {

        // 设置无力标志
        YawCtrl.nopowerflag = 1; // 1表示无力，电机输出为零
        PitchCtrl.nopowerflag = 1;

        return;
    }

	// LYL
    // 状态转换处理：从无力模式切换到其他模式
    if (lastMode == RobotState_e_Powerless)
    {
        // 初始化目标角度为当前位置，避免跳变
        PitchCtrl.want_AbsoluteAngleCtrl = PitchCtrl.nowAbsoluteAngle;
        YawCtrl.want_AbsoluteAngleCtrl = YawCtrl.nowAbsoluteAngle;

        // 清除无力标志
        YawCtrl.nopowerflag = 0;
        PitchCtrl.nopowerflag = 0;
    }
	
	
    // 自动控制模式

	delta_pitch = -(float)(nx_p->pitch)*3.1415926f/32767-PitchCtrl.nowAbsoluteAngle;
    //算增量
    delta_yaw = (float)(nx_p->yaw)*3.1415926f/32767-YawCtrl.nowAbsoluteAngle;
//	if(delta_pitch <= target && delta_pitch >= target)
//	{
//		delta_pitch = 0;
//	}
//	
//	if(delta_yaw <= target && delta_yaw >= -target)
//	{
//		delta_yaw = 0;
//	}
	// LYL
    if (robotIsAuto() || rc_p->rc.s[1]==2 )
    {
		// LYL
        if (data_ready == 1 )//&& nx_p->mode != 0)
        {
            
            // delta 以当前实测角度为基准，不能累加到旧目标，否则会重复叠加跟踪误差。
            // Pitch轴自动控制
            PitchCtrl.want_AbsoluteAngleCtrl = PitchCtrl.nowAbsoluteAngle + delta_pitch;

            // Yaw轴自动控制
            YawCtrl.want_AbsoluteAngleCtrl = YawCtrl.nowAbsoluteAngle + delta_yaw;
				
            data_ready = 0;
        }
    } 
	else
	{
        // Pitch轴手动控制（积分累加方式）
        PitchCtrl.want_AbsoluteAngleCtrl +=
            1 * pitch_channel * PITCH_RC_SEN        // 遥控器控制
            - 10 * rc_p->mouse.y * PITCH_MOUSE_SEN; // 鼠标控制

        // Yaw轴手动控制（积分累加方式）
        YawCtrl.want_AbsoluteAngleCtrl +=
            5 * yaw_channel * YAW_RC_SEN          // 遥控器控制
            + 20 * rc_p->mouse.x * YAW_MOUSE_SEN; // 鼠标控制

        // Yaw轴角度范围限制（保持在[-π, π]之间）
        if (YawCtrl.want_AbsoluteAngleCtrl >= 3.086f)
        {
            YawCtrl.want_AbsoluteAngleCtrl -= 2 * PI;
        }
        if (YawCtrl.want_AbsoluteAngleCtrl <= -3.196f)
        {
            YawCtrl.want_AbsoluteAngleCtrl += 2 * PI;
        }
    }

    // 记录当前模式，用于下一次状态转换判断
    lastMode = *robotMode;
}

/**
 * @brief PID计算函数（包含位置环和速度环）
 */
void unitPID_calc(void)
{

    if (PitchCtrl.want_AbsoluteAngleCtrl > PITCH_MAX_ANGLE)
    {
        PitchCtrl.want_AbsoluteAngleCtrl = PITCH_MAX_ANGLE;
    }
    else if (PitchCtrl.want_AbsoluteAngleCtrl < PITCH_MIN_ANGLE)
    {
        PitchCtrl.want_AbsoluteAngleCtrl = PITCH_MIN_ANGLE;
    }

    // Yaw轴位置环PID计算
    gimbal_PID_calc(&(YawCtrl.agl_pid), YawCtrl.nowAbsoluteAngle, YawCtrl.want_AbsoluteAngleCtrl);
    // Yaw轴速度环PID计算
    PID_calc(&(YawCtrl.spd_pid), YawCtrl.radspeed, YawCtrl.agl_pid.out);
    YawCtrl.Volt_out = YawCtrl.spd_pid.out;

    // Yaw轴无力标志检查
    if (YawCtrl.nopowerflag)
    {
        YawCtrl.Volt_out = 0;
    }

    // Pitch轴位置环PID计算
    gimbal_PID_calc(&(PitchCtrl.agl_pid), PitchCtrl.nowAbsoluteAngle, PitchCtrl.want_AbsoluteAngleCtrl);
    // Pitch轴速度环PID计算
    PID_calc(&(PitchCtrl.spd_pid), PitchCtrl.radspeed, PitchCtrl.agl_pid.out);
    PitchCtrl.Volt_out = -PitchCtrl.spd_pid.out;//changedv2

    // Pitch轴无力标志检查
    if (PitchCtrl.nopowerflag)
    {
        PitchCtrl.Volt_out = 0;
    }
}

/**
 * @brief 获取Yaw轴控制结构体指针
 */
const struct GimbalMotorCtrl_s *get_wantedyaw_point(void)
{
    return &YawCtrl;
}
/**
 * @brief 获取Pitch轴控制结构体指针
 */
const struct GimbalMotorCtrl_s *get_wantedpitch_point(void)
{
    return &PitchCtrl;
}

// 获取Yaw轴当前角度
fp32 get_yaw_current_angle(void)
{
    return YawCtrl.nowAbsoluteAngle;
}

// 获取Yaw轴当前角速度（弧度/秒）
fp32 get_yaw_current_speed(void)
{
    return YawCtrl.radspeed * 1000.0f; // 转换为弧度/秒
}

// 获取Pitch轴当前角度
fp32 get_pitch_current_angle(void)
{
    return PitchCtrl.nowAbsoluteAngle;
}

// 获取Pitch轴当前角速度（弧度/秒）
fp32 get_pitch_current_speed(void)
{
    return PitchCtrl.radspeed * 1000.0f; // 转换为弧度/秒
}
