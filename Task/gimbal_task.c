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
#include "tuning_protocol.h"

// 任务周期
#define GIMBAL_TASK_CTRL_TIME 2

// 速度环PID参数
#define PITCH_SPD_KP 4000000.0f
/* 保持 0，与车上基线一致。积分路径已就绪，上车验证时在调试器里改
 * PitchCtrl.spd_pid.Ki；Ki 按每 2 ms 一次的单位，按实际控制周期折算。 */
#define PITCH_SPD_KI 000.0f
#define PITCH_SPD_KD 1000.0f
/* 启动延时或调试暂停后不补算长时间积分，正常抖动按实际经过时间折算。 */
#define PITCH_I_MAX_DT_MS 10U
/* 积分分离：角度误差超过此值时暂停积分并保留已有积分，防止大角度转动中积分攒满造成超调。
 * 必须大于 Ki=0 时观察到的静差，否则积分永远不会介入。单位 rad，约 5.7°。 */
#define PITCH_I_SEPARATION_RAD_DEFAULT 0.1f

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
#define PITCH_VOLT_MAX_OUT 24000.0f //GM6020手册上写的是25000，软件要比硬件严格一点
#define PITCH_VOLT_MAX_IOUT 3000.0f
#define YAW_VOLT_MAX_OUT 30000.0f
#define YAW_VOLT_MAX_IOUT 10000.0f

// Pitch重力补偿参数，输出单位与速度环PID相同，幅值和方向需实车标定
#define PITCH_GRAVITY_GAIN_DEFAULT    -800.0f    // 补偿幅值
#define PITCH_GRAVITY_OFFSET_DEFAULT  0.0f    // 调整在哪个俯仰角的补偿最大，默认0=水平
#define PITCH_GRAVITY_MAX_OUT         5000.0f // 补偿自身的输出限幅
#define PITCH_GRAVITY_SLEW_PER_SECOND 20000.0f // 补偿每秒最多变化的控制量

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

volatile pitch_gravity_config_t pitch_gravity_config = {
    PITCH_GRAVITY_GAIN_DEFAULT,
    PITCH_GRAVITY_OFFSET_DEFAULT
};
volatile pitch_control_debug_t pitch_control_debug;
volatile fp32 pitch_i_separation_rad = PITCH_I_SEPARATION_RAD_DEFAULT; // 可在调试器里修改
static fp32 pitch_gravity_output; // 上一周期的补偿，用于限制本周期的变化速度

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
            pitch_control_debug.motor_cmd = 0;
            pitch_control_debug.control_enabled = 0;
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
    CAN_Send_to_MCU frame;
    uint8_t frame_source;
    const uint8_t automatic = robotIsAuto() || rc_p->rc.s[1] == 2;
    /* 退出自瞄或遥控掉线后撤掉串口来源，不能在重新使能时消费旧测试目标。 */
    if (!automatic || !rc_online || *robotMode == RobotState_e_Powerless)
        NX_SelectSource(NX_SOURCE_CAN);
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

        // 无力期间也要记录模式，否则恢复上电时不会把目标重置到当前位置，云台会跳回停机前的目标
        lastMode = *robotMode;
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
    if (automatic)
    {
		// LYL
        if (NX_TakeControlFrame(&frame, &frame_source))
        {
            delta_pitch = -(float)frame.pitch * 3.1415926f / 32767 - PitchCtrl.nowAbsoluteAngle;
            delta_yaw = (float)frame.yaw * 3.1415926f / 32767 - YawCtrl.nowAbsoluteAngle;
            
            // delta 以当前实测角度为基准，不能累加到旧目标，否则会重复叠加跟踪误差。
            // Pitch轴自动控制
            PitchCtrl.want_AbsoluteAngleCtrl = PitchCtrl.nowAbsoluteAngle + delta_pitch;

            // Yaw轴自动控制
            if (frame_source == NX_SOURCE_CAN)
                YawCtrl.want_AbsoluteAngleCtrl = YawCtrl.nowAbsoluteAngle + delta_yaw;
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

static uint8_t gimbal_tuning_is_enabled(void)
{
    return rc_online && robotMode != NULL && rc_p != NULL &&
           *robotMode != RobotState_e_Powerless &&
           (robotIsAuto() || rc_p->rc.s[1] == 2);
}

void gimbal_tuning_get_state(pitch_tuning_state_t *state)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    state->tick_ms = xTaskGetTickCount();
    state->gravity = pitch_gravity_config;
    state->control = pitch_control_debug;
    state->speed_ki = PitchCtrl.spd_pid.Ki;
    state->source = NX_GetSource();
    state->remote_online = rc_online;
    state->automatic = gimbal_tuning_is_enabled();
    __set_PRIMASK(key);
}

void gimbal_tuning_get_limits(pitch_tuning_limits_t *limits)
{
    limits->pitch_min_rad = PITCH_MIN_ANGLE;
    limits->pitch_max_rad = PITCH_MAX_ANGLE;
    limits->gain_max = PITCH_GRAVITY_MAX_OUT;
    limits->offset_max_rad = PI;
    limits->gravity_max_out = PITCH_GRAVITY_MAX_OUT;
    limits->total_max_out = PITCH_VOLT_MAX_OUT;
}

uint8_t gimbal_tuning_set_gravity(fp32 gain, fp32 offset_rad)
{
    uint32_t key;
    if (!isfinite(gain) || !isfinite(offset_rad) ||
        fabsf(gain) > PITCH_GRAVITY_MAX_OUT || fabsf(offset_rad) > PI)
        return TUNING_OUT_OF_RANGE;
    key = __get_PRIMASK();
    __disable_irq();
    pitch_gravity_config.gain = gain;
    pitch_gravity_config.offset_rad = offset_rad;
    __set_PRIMASK(key);
    return TUNING_OK;
}

uint8_t gimbal_tuning_set_source(uint8_t source)
{
    if (source != NX_SOURCE_CAN && source != NX_SOURCE_UART)
        return TUNING_BAD_PARAMETER;
    if (source == NX_SOURCE_UART && !gimbal_tuning_is_enabled())
        return TUNING_NOT_ENABLED;
    NX_SelectSource(source);
    return TUNING_OK;
}

uint8_t gimbal_tuning_submit_vision(const uint8_t payload[8])
{
    int16_t pitch_raw;
    fp32 pitch_rad;
    if (!gimbal_tuning_is_enabled())
        return TUNING_NOT_ENABLED;
    if (NX_GetSource() != NX_SOURCE_UART)
        return TUNING_WRONG_SOURCE;
    /* 模拟端只提供 Pitch 目标，不接受开火请求；Yaw 目标由原状态机保持。 */
    if (payload[0] != 1U || payload[5] != 0U || payload[6] != 0U || payload[7] != 0U)
        return TUNING_BAD_PARAMETER;
    pitch_raw = (int16_t)((uint16_t)payload[3] | ((uint16_t)payload[4] << 8));
    pitch_rad = -(fp32)pitch_raw * 3.1415926f / 32767;
    if (pitch_rad < PITCH_MIN_ANGLE || pitch_rad > PITCH_MAX_ANGLE)
        return TUNING_OUT_OF_RANGE;
    return NX_ReceiveFrame(payload, NX_SOURCE_UART) ? TUNING_OK : TUNING_WRONG_SOURCE;
}

/**
 * @brief 根据当前实际角度计算Pitch重力补偿，并限制补偿的大小和变化速度
 *
 * 手动、自瞄和静止保持共用，不按追踪方向改变补偿方向。
 * 无力或遥控掉线时，由unitPID_calc立即清零补偿。
 */
static fp32 calcPitchGravity(pitch_control_debug_t *debug)
{
    const fp32 gain = pitch_gravity_config.gain;
    const fp32 offset = pitch_gravity_config.offset_rad;
    const fp32 angle = PitchCtrl.nowAbsoluteAngle;
    const uint32_t dt = PitchCtrl.nowTime - PitchCtrl.lastTime; // 单位ms
    fp32 gravityTarget = 0.0f;

    // 参数或角度无效时，立即撤掉补偿
    if (!isfinite(gain) || !isfinite(offset) ||
        !isfinite(angle) || fabsf(offset) > PI)
    {
        pitch_gravity_output = 0.0f;
        debug->gravity_config_invalid = 1;
        return 0.0f;
    }

    // 计算目标补偿，公式中的角度均为rad；gain为0时，目标补偿也为0
    if (gain != 0.0f)
    {
        gravityTarget = gain * cosf(angle + offset);
    }
    debug->gravity_limited = fabsf(gravityTarget) > PITCH_GRAVITY_MAX_OUT;
    range_limit_inside(gravityTarget, PITCH_GRAVITY_MAX_OUT);

    // 逐步靠近目标补偿，正常2 ms周期内每次最多变化12
    // gain改为0也逐步退回零；周期为0或过长时，保持上次补偿
    if (dt > 0U && dt <= PITCH_I_MAX_DT_MS)
    {
        const fp32 maxStep = PITCH_GRAVITY_SLEW_PER_SECOND * (fp32)dt * 0.001f;
        fp32 step = gravityTarget - pitch_gravity_output;
        range_limit_inside(step, maxStep);
        pitch_gravity_output += step;
    }

    // 记录目标补偿和本周期实际采用的补偿
    debug->gravity_target_out = gravityTarget;
    debug->gravity_out = pitch_gravity_output;
    return pitch_gravity_output;
}

/**
 * @brief 合并Pitch速度环PID和重力补偿，处理积分及总输出限幅
 * @param gravityOut 本周期的重力补偿
 * @param lastIout 本次PID计算前的积分，用于撤销加重超限的积分
 * @return 限幅后的总输出，尚未转换电机方向
 */
static fp32 limitPitchOutput(fp32 gravityOut, fp32 lastIout, pitch_control_debug_t *debug)
{
    pid_type_def *pid = &PitchCtrl.spd_pid;
    fp32 totalOut = pid->Pout + pid->Iout + pid->Dout + gravityOut;
    const fp32 integralChange = pid->Iout - lastIout;

    // 总输出超限时，撤销继续加重超限的积分；反向释放的积分仍保留
    if ((totalOut > pid->max_out && integralChange > 0.0f) ||
        (totalOut < -pid->max_out && integralChange < 0.0f))
    {
        pid->Iout = lastIout;
        debug->integral_blocked = 1;
    }

    // 积分可能被撤销，重新计算PID输出；pid->out仍只记录PID自身的限幅结果
    const fp32 pidOut = pid->Pout + pid->Iout + pid->Dout;
    pid->out = pidOut;
    range_limit_inside(pid->out, pid->max_out);

    // 总输出使用未限幅的PID值，先加重力补偿再限幅，不能直接用pid->out
    totalOut = pidOut + gravityOut;
    const fp32 totalBeforeLimit = totalOut;
    range_limit_inside(totalOut, pid->max_out);

    // 记录计算结果和各自是否到达上限，调试数据不参与控制计算
    debug->p_out = pid->Pout;
    debug->i_out = pid->Iout;
    debug->d_out = pid->Dout;
    debug->pid_unclamped_out = pidOut;
    debug->total_unclamped_out = totalBeforeLimit;
    debug->total_out = totalOut;
    debug->integral_limited = fabsf(pid->Iout) >= pid->max_iout;
    debug->pid_saturated = fabsf(pidOut) >= pid->max_out;
    debug->total_saturated = fabsf(totalBeforeLimit) >= pid->max_out;
    return totalOut;
}

/**
 * @brief 计算Pitch速度环PID，再合并重力补偿并处理限幅
 *
 * 保留原 P/D 与速度死区；不改变其他轴的 PID_calc。
 * Ki 沿用每 2 ms 一次的参数单位，按控制环时间缩放，与视觉收包次数无关。
 * 角度误差超过 pitch_i_separation_rad 时暂停积分，已有积分保留。
 */
static fp32 pitch_speed_pid_calc(fp32 gravity_out, pitch_control_debug_t *debug)
{
    pid_type_def *pid = &PitchCtrl.spd_pid;
    const fp32 previous_i = pid->Iout;
    const fp32 nominal_ki = pid->Ki;
    const uint32_t dt_ms = PitchCtrl.nowTime - PitchCtrl.lastTime;
    // 本周期角度环误差，gimbal_PID_calc 已归一化到 [-PI, PI]
    const uint8_t separated = fabsf(PitchCtrl.agl_pid.error[0]) > pitch_i_separation_rad;

    debug->integral_separated = separated;
    pid->Ki = (dt_ms > 0U && dt_ms <= PITCH_I_MAX_DT_MS && !separated)
                  ? nominal_ki * (fp32)dt_ms / GIMBAL_TASK_CTRL_TIME
                  : 0.0f;
    PID_calc(pid, PitchCtrl.radspeed, PitchCtrl.agl_pid.out);
    pid->Ki = nominal_ki;

    return limitPitchOutput(gravity_out, previous_i, debug);
}

/**
 * @brief PID计算函数（包含位置环和速度环）
 */
void unitPID_calc(void)
{
    // 每周期重新记录，停机时未计算的输出和标志保持为零
    pitch_control_debug_t debug = {0};
    debug.requested_angle_rad = PitchCtrl.want_AbsoluteAngleCtrl;
    if (PitchCtrl.want_AbsoluteAngleCtrl > PITCH_MAX_ANGLE)
    {
        PitchCtrl.want_AbsoluteAngleCtrl = PITCH_MAX_ANGLE;
        debug.target_limited = 1;
    }
    else if (PitchCtrl.want_AbsoluteAngleCtrl < PITCH_MIN_ANGLE)
    {
        PitchCtrl.want_AbsoluteAngleCtrl = PITCH_MIN_ANGLE;
        debug.target_limited = 1;
    }

    debug.target_angle_rad = PitchCtrl.want_AbsoluteAngleCtrl;
    debug.actual_angle_rad = PitchCtrl.nowAbsoluteAngle;
    debug.speed_rad_s = PitchCtrl.radspeed * 1000.0f;
    debug.angle_error_rad = radFormat(debug.target_angle_rad - debug.actual_angle_rad);

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

    /* 无力或遥控掉线时不积累 Pitch 积分，避免恢复使能后带入停机误差。 */
    if (PitchCtrl.nopowerflag || rc_online == 0)
    {
        PID_clear(&(PitchCtrl.spd_pid));
        PID_clear(&(PitchCtrl.agl_pid));
        pitch_gravity_output = 0.0f;
        PitchCtrl.Volt_out = 0;
        pitch_control_debug = debug;
        return;
    }

    // Pitch轴位置环PID计算
    gimbal_PID_calc(&(PitchCtrl.agl_pid), PitchCtrl.nowAbsoluteAngle, PitchCtrl.want_AbsoluteAngleCtrl);
    // 计算重力补偿，由速度环合并PID输出并处理限幅
    const fp32 gravity_out = calcPitchGravity(&debug);
    const fp32 total_out = pitch_speed_pid_calc(gravity_out, &debug);

    // 转换电机方向，并保存本周期的调试记录
    PitchCtrl.Volt_out = (int16_t)(-total_out);//changedv2
    debug.motor_cmd = PitchCtrl.Volt_out;
    debug.control_enabled = 1;
    pitch_control_debug = debug;
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
