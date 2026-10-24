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
/* 上电积分增益；串口只修改运行值，重启恢复此值。
 * Ki 按每 2 ms 一次的单位，按实际控制周期折算。 */
#define PITCH_SPD_KI 6000.0f
#define PITCH_SPD_KI_MAX 8000.0f // 串口实验范围上限，不代表已验证的稳定增益
#define PITCH_SPD_KD 1000.0f
/* 启动延时或调试暂停后不补算长时间积分，正常抖动按实际经过时间折算。 */
#define PITCH_I_MAX_DT_MS 10U
/* 积分分离：角度误差超过此值时暂停积分并保留已有积分，防止大角度转动中积分攒满造成超调。
 * 必须大于 Ki=0 时观察到的静差，否则积分永远不会介入。单位 rad，约 5.7°。 */
#define PITCH_I_SEPARATION_RAD_DEFAULT 0.1f

#define YAW_SPD_KP 10000000.0f
//数值大是因为角速度计算那里的dt是ms，PID内部沿用rad/ms，要乘1000
//实际除回去就是10000，速度环控制的是电机电压值，范围为0~25000，所以合理
#define YAW_SPD_KI 0	
#define YAW_SPD_KD 0

// 位置环PID参数
#define PITCH_AGL_KP 0.015f
#define PITCH_AGL_KI 0
#define PITCH_AGL_KD 0.01f

#define YAW_AGL_KP 0.018f
#define YAW_AGL_KI 0.00f
#define YAW_AGL_KD 0.00000f

// 功率保护参数
#define PITCH_VOLT_MAX_OUT 24000.0f //GM6020手册上写的是25000，软件要比硬件严格一点
#define PITCH_VOLT_MAX_IOUT 3000.0f
#define YAW_VOLT_MAX_OUT 30000.0f
#define YAW_VOLT_MAX_IOUT 10000.0f

// Pitch重力补偿参数，输出单位与速度环PID相同，幅值和方向需实车标定
#define PITCH_GRAVITY_GAIN_DEFAULT    2500.0f    // 补偿幅值
#define PITCH_GRAVITY_OFFSET_DEFAULT  1.16f    // 调整在哪个俯仰角的补偿最大，默认0=水平
#define PITCH_GRAVITY_MAX_OUT         5000.0f // 补偿自身的输出限幅
#define PITCH_GRAVITY_SLEW_PER_SECOND 20000.0f // 补偿每秒最多变化的控制量

// 静摩擦补偿（启动补足），Pitch和Yaw共用同一套判断，输出单位与速度环PID相同，门槛需实车测定
/* 确认停住、离目标还远、PID方向对但推不动时，把PID输出补到门槛值。
 * 补的是差额（门槛 - PID），不是在PID上再加一个固定值。门槛为0即关闭该方向。 */
#define FRICTION_MAX_OUT                 5000.0f   // 门槛允许的最大值，超过视为填错，按关闭处理
#define FRICTION_STILL_MS                100U      // 连续静止满这么久才算停住，单位ms
#define FRICTION_STILL_DRIFT_RAD         0.001745f // 约0.1°，静止计时期间允许的角度漂移
#define PITCH_FRICTION_UP_OUT_DEFAULT    750.0f    // 抬头方向的门槛，实车初值
#define PITCH_FRICTION_DOWN_OUT_DEFAULT  750.0f    // 低头方向的门槛，实车初值
#define PITCH_FRICTION_ERROR_RAD_DEFAULT 0.0087f   // 约0.5°，离目标比它近时不补足
#define YAW_FRICTION_POS_OUT_DEFAULT     1000.0f      // Yaw角度增大方向的门槛，默认关闭；建议从600开始试
#define YAW_FRICTION_NEG_OUT_DEFAULT     1000.0f      // Yaw角度减小方向的门槛，默认关闭；建议从600开始试
#define YAW_FRICTION_ERROR_RAD_DEFAULT   0.0087f   // 约0.5°，离目标比它近时不补足

// Yaw速度前馈：目标怎么动是已知的，直接把目标角速度加到速度环目标上，不等误差出现
/* 速度环目标 = 位置环输出 + 增益 × 目标角速度。
 * 视觉来源的目标角速度用相邻帧估算，跳变会被放大，所以增益不给满（实车约0.66）。 */
#define YAW_FF_SPEED_GAIN_DEFAULT      0.7f    // 速度前馈增益，调到1为完全前馈
#define YAW_FF_GAIN_MAX                2.0f    // 增益超过它视为填错，按关闭处理
#define YAW_FF_SPEED_MAX               10.0f   // 估算的目标角速度超过约573°/s视为数据错误，按0处理
#define YAW_FF_FRAME_DT_MIN_MS         30U     // 两帧间隔小于它（连发帧或成串到达），不拿来估速度，避免短间隔把跳变放大成很大的速度
#define YAW_FF_FRAME_DT_MAX_MS         100U    // 两帧间隔大于它（中间断过），速度按0重新开始

// 开火门控：视觉允许开火、视觉帧没过期、云台已对准且保持一小段时间，才允许拨弹
/* 允许误差（rad）= 允许偏离的半宽（m）/ 距离（m）- 散布角（rad），再做上下限。
 * 半宽 = 装甲板实测半宽 × 0.7余量；视觉预测误差和散布也会让弹丸偏，所以不用满。 */
#define FIRE_YAW_HALF_M_DEFAULT    0.0675f  // 装甲板横宽13.5cm的一半6.75cm
#define FIRE_PITCH_HALF_M_DEFAULT  0.0615f  // 装甲板竖向投影约12.3cm的一半6.15cm
#define FIRE_SPREAD_RAD_DEFAULT    0.006f    // 发射散布角，实测前先为0
#define FIRE_MIN_RAD_DEFAULT       0.0052f // 约0.3°，算出来比它还小说明太远打不准，不开火
#define FIRE_MAX_RAD_DEFAULT       0.0224f // 约1.28°，算出来比它还大说明太近打不准，不开火
#define FIRE_HOLD_MS_DEFAULT       10U     // 误差连续在范围内满这么久才开火，必须小于视觉帧间隔约42ms
#define FIRE_FRAME_TIMEOUT_MS      100U    // 视觉帧超过这么久没更新，视为视觉掉线，不开火
#define FIRE_DISTANCE_MIN_CM       30U     // 距离小于30cm视为无效（包括0，即视觉没给距离）
#define FIRE_DISTANCE_MAX_CM       1500U   // 距离超过15m视为数据错误
#define FIRE_DISTANCE_DEFAULT_M    3.0f    // 距离无效时按3m算允许误差，视觉还没下发距离时也能开火

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
#define PITCH_MIN_ANGLE -0.31f //备步实测-31.8°，软件限幅要比机械严格一点

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

    // 电机反馈转速（整数rpm），Pitch静摩擦补偿用它判断是否停住
    const int16_t *SpeedRpmPoint;

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

// 静摩擦补偿门槛和死区，可在调试器里修改
volatile fp32 pitch_friction_up_out = PITCH_FRICTION_UP_OUT_DEFAULT;
volatile fp32 pitch_friction_down_out = PITCH_FRICTION_DOWN_OUT_DEFAULT;
volatile fp32 pitch_friction_error_rad = PITCH_FRICTION_ERROR_RAD_DEFAULT;
volatile fp32 yaw_friction_pos_out = YAW_FRICTION_POS_OUT_DEFAULT;
volatile fp32 yaw_friction_neg_out = YAW_FRICTION_NEG_OUT_DEFAULT;
volatile fp32 yaw_friction_error_rad = YAW_FRICTION_ERROR_RAD_DEFAULT;

// 静摩擦补偿的运行状态，每个轴一份；后三项每周期刷新，可在调试器里观察
typedef struct
{
    uint8_t still_valid;        // 0表示还没开始静止计时
    uint32_t still_start_time;  // 本次静止从什么时候开始，ms
    fp32 still_start_angle;     // 本次静止开始时的角度，rad
    uint8_t still;              // 本周期已确认停住
    uint8_t active;             // 本周期补偿生效
    fp32 out;                   // 本周期补上的差额，不补时为0
} friction_state_t;

static friction_state_t pitch_friction_state;
static friction_state_t yaw_friction_state;

// 开火门控参数，可在调试器里修改
volatile fp32 fire_yaw_half_m = FIRE_YAW_HALF_M_DEFAULT;
volatile fp32 fire_pitch_half_m = FIRE_PITCH_HALF_M_DEFAULT;
volatile fp32 fire_spread_rad = FIRE_SPREAD_RAD_DEFAULT;
volatile fp32 fire_min_rad = FIRE_MIN_RAD_DEFAULT;
volatile fp32 fire_max_rad = FIRE_MAX_RAD_DEFAULT;
volatile uint32_t fire_hold_ms = FIRE_HOLD_MS_DEFAULT;
volatile fire_gate_debug_t fire_gate_debug; // 每周期刷新，只供观察
// 最近一帧视觉数据中与开火有关的部分
static uint8_t fire_frame_valid;     // 0表示还没收到过视觉帧
static uint32_t fire_frame_time;     // 收到这一帧的时间，ms
static uint8_t fire_frame_source;    // 这一帧来自CAN还是串口模拟
static uint8_t fire_frame_fire;      // 视觉的fire字节，1表示视觉允许开火
static uint16_t fire_frame_rev;      // 视觉的rev字段：bit0只瞄准不发射，bit1~15距离（cm）
// 对准保持计时
static uint8_t fire_hold_valid;      // 0表示还没开始计时
static uint32_t fire_hold_start;     // 本次对准从什么时候开始，ms
static volatile uint8_t fire_allowed; // 本周期是否允许开火，发射任务读取

// PID参数数组
static fp32 pitchSpdPIDco[3] = {PITCH_SPD_KP, PITCH_SPD_KI, PITCH_SPD_KD};
static fp32 yawSpdPIDco[3] = {YAW_SPD_KP, YAW_SPD_KI, YAW_SPD_KD};
static fp32 pitchAglPIDco[3] = {PITCH_AGL_KP, PITCH_AGL_KI, PITCH_AGL_KD};
static fp32 yawAglPIDco[3] = {YAW_AGL_KP, YAW_AGL_KI, YAW_AGL_KD};

// 高频测试独立保存状态，不修改PID参数，不经过CAN开火请求。
typedef struct
{
    uint8_t active;
    fp32 center_rad;
    fp32 amplitude_rad;
    fp32 frequency_hz;
    uint32_t duration_ms;
    uint32_t start_ms;
    uint32_t keepalive_ms;
} yaw_wave_state_t;
static yaw_wave_state_t yaw_wave;

// Yaw目标前馈参数，可在调试器里修改
volatile fp32 yaw_ff_speed_gain = YAW_FF_SPEED_GAIN_DEFAULT;
volatile yaw_ff_debug_t yaw_ff_debug; // 每周期刷新，只供观察
// 本周期目标的角速度（rad/s），由getControlAngles每周期重新给出
static fp32 yaw_target_speed;
static uint8_t yaw_target_source;
// 最近一帧视觉Yaw目标，用来估算目标角速度
static uint8_t yaw_frame_valid;      // 0表示还没收到过帧
static fp32 yaw_frame_angle;         // 这一帧的Yaw目标，rad
static uint32_t yaw_frame_time;      // 收到这一帧的时间，ms
static fp32 yaw_frame_speed;         // 由这一帧和上一帧估算的目标角速度，rad/s
static uint32_t yaw_frame_dt;        // 这一帧和上一帧的间隔，ms
// 本周期速度环的目标（位置环输出 + 速度前馈），rad/ms，供调参读取
static fp32 yaw_speed_setpoint;

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
static void recordFireFrame(const CAN_Send_to_MCU *frame, uint8_t source);
static void updateFireGate(void);
static uint8_t gimbal_tuning_is_enabled(void);
static void updateYawWave(void);
static void recordYawFrame(fp32 targetAngle);
static void updateYawVisionMotion(uint8_t automatic);

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

        // 判断是否允许开火，要用unitPID_calc限幅后的Pitch目标
        updateFireGate();

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

    // 初始化转速指针，静摩擦补偿用它判断是否停住
    PitchCtrl.SpeedRpmPoint = &(get_pitch_gimbal_motor_measure_point()->speed_rpm);
    YawCtrl.SpeedRpmPoint = &(get_yaw_gimbal_motor_measure_point()->speed_rpm);

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
    // 目标运动每周期重新给出；手动、无力等情况下保持为0，前馈不起作用
    yaw_target_speed = 0.0f;
    yaw_target_source = YAW_TARGET_SOURCE_NONE;
    /* 退出自瞄或遥控掉线后撤掉串口来源，不能在重新使能时消费旧测试目标。 */
    if (!automatic || !rc_online || *robotMode == RobotState_e_Powerless)
    {
        NX_SelectSource(NX_SOURCE_CAN);
        yaw_wave.active = 0;
        yaw_frame_valid = 0;
    }
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
            // 记下这一帧的开火请求、距离和到达时间，供开火门控使用
            recordFireFrame(&frame, frame_source);

            delta_pitch = -(float)frame.pitch * 3.1415926f / 32767 - PitchCtrl.nowAbsoluteAngle;
            delta_yaw = (float)frame.yaw * 3.1415926f / 32767 - YawCtrl.nowAbsoluteAngle;
            
            // delta 以当前实测角度为基准，不能累加到旧目标，否则会重复叠加跟踪误差。
            // Pitch轴自动控制
            PitchCtrl.want_AbsoluteAngleCtrl = PitchCtrl.nowAbsoluteAngle + delta_pitch;

            // Yaw轴自动控制
            // CAN和串口模拟共用同一绝对角度载荷，不累加旧目标。
            YawCtrl.want_AbsoluteAngleCtrl = YawCtrl.nowAbsoluteAngle + delta_yaw;

            // 记下这一帧的Yaw目标，和上一帧比较估算目标角速度
            recordYawFrame(YawCtrl.want_AbsoluteAngleCtrl);
        }
        // 视觉目标的角速度和帧间插值
        updateYawVisionMotion(automatic);
        // 高频模式在控制周期内更新，避免24 Hz目标下发造成混叠。
        // 波形测试时，目标和目标运动都以波形为准，覆盖上面视觉的结果
        updateYawWave();
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

#if TUNING_DIAGNOSTICS_ENABLED && TUNING_VOFA_TX_ENABLED
/* 只采集观测值；由快照函数在已有临界区内调用，不写控制状态。 */
static void getTuningDiagnostics(pitch_tuning_state_t *state)
{
    state->pitch_encoder_angle_rad = 0;
    // 调参任务可能早于云台初始化，编码器指针就绪后才读取。
    if (PitchCtrl.ECDPoint != NULL)
    {
        state->pitch_encoder_angle_rad = ECD2Rad(*PitchCtrl.ECDPoint);
    }

    const RC_ctrl_t *remote = get_remote_control_point();
    state->remote_left_switch = remote->rc.s[1];
    state->friction_enabled = get_fric();
    for (uint8_t motor = 0; motor < 2; motor++)
    {
        const motor_measure_t *measure = get_shoot_motor_measure_point(motor);
        state->friction_speed_rpm[motor] = measure->speed_rpm;
    }
    INS_GetGyroStats(state->gyro_mean_rad_s, state->gyro_std_rad_s);
}
#endif

void gimbal_tuning_get_state(pitch_tuning_state_t *state)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    state->tick_ms = xTaskGetTickCount();
    state->gravity = pitch_gravity_config;
    state->control = pitch_control_debug;
    state->fire_gate = fire_gate_debug;
    state->speed_ki = PitchCtrl.spd_pid.Ki;
    state->source = NX_GetSource();
    state->remote_online = rc_online;
    state->automatic = gimbal_tuning_is_enabled();
    state->vision_online = NX_VisionOnline();

    // 只复制Yaw的控制结果，不在调参任务中重新执行PID。
    state->yaw.target_angle_rad = YawCtrl.want_AbsoluteAngleCtrl;
    state->yaw.actual_angle_rad = YawCtrl.nowAbsoluteAngle;
    // 控制内部使用rad/ms；上传前统一转换为rad/s。
    state->yaw.target_speed_rad_s = yaw_speed_setpoint * 1000.0f; // 位置环输出 + 速度前馈
    state->yaw.speed_rad_s = YawCtrl.radspeed * 1000.0f;
    state->yaw.motor_cmd = YawCtrl.Volt_out;
#if TUNING_DIAGNOSTICS_ENABLED && TUNING_VOFA_TX_ENABLED
    getTuningDiagnostics(state);
#endif
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
    limits->speed_ki_max = PITCH_SPD_KI_MAX;
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

/* 串口任务低于云台任务优先级；临界区内一起提交，云台不会读到半组参数。
 * 非零 Ki 只改变后续积分速度，保留已有 I 输出；Ki=0 明确撤掉已有积分。
 */
uint8_t gimbal_tuning_set_parameters(fp32 gain, fp32 offset_rad, fp32 speed_ki)
{
    uint32_t key;
    uint8_t status;
    if (!isfinite(speed_ki) || speed_ki < 0.0f || speed_ki > PITCH_SPD_KI_MAX)
        return TUNING_OUT_OF_RANGE;
    key = __get_PRIMASK();
    __disable_irq();
    status = gimbal_tuning_set_gravity(gain, offset_rad);
    if (status == TUNING_OK)
    {
        PitchCtrl.spd_pid.Ki = speed_ki;
        if (speed_ki == 0.0f)
            PitchCtrl.spd_pid.Iout = 0.0f;
    }
    __set_PRIMASK(key);
    return status;
}

uint8_t gimbal_tuning_set_source(uint8_t source)
{
    if (source != NX_SOURCE_CAN && source != NX_SOURCE_UART)
        return TUNING_BAD_PARAMETER;
    if (source == NX_SOURCE_UART && !gimbal_tuning_is_enabled())
        return TUNING_NOT_ENABLED;
    // 手动切换来源会结束原测试，重新开始必须明确发开始命令。
    gimbal_tuning_stop_yaw_wave();
    NX_SelectSource(source);
    return TUNING_OK;
}

uint8_t gimbal_tuning_submit_vision(const uint8_t payload[8])
{
    int16_t pitch_raw;
    fp32 pitch_rad;
    uint8_t accepted;
    uint32_t key;
    if (!gimbal_tuning_is_enabled())
        return TUNING_NOT_ENABLED;
    if (NX_GetSource() != NX_SOURCE_UART)
        return TUNING_WRONG_SOURCE;
    /* 串口模拟可提供双轴绝对目标，但不接受开火请求。 */
    if (payload[0] != 1U || payload[5] != 0U || payload[6] != 0U || payload[7] != 0U)
        return TUNING_BAD_PARAMETER;
    pitch_raw = (int16_t)((uint16_t)payload[3] | ((uint16_t)payload[4] << 8));
    pitch_rad = -(fp32)pitch_raw * 3.1415926f / 32767;
    if (pitch_rad < PITCH_MIN_ANGLE || pitch_rad > PITCH_MAX_ANGLE)
        return TUNING_OUT_OF_RANGE;
    key = __get_PRIMASK();
    __disable_irq();
    accepted = NX_ReceiveFrame(payload, NX_SOURCE_UART);
    // 手动目标应覆盖波形测试，不能下一周期又被旧波形改回去。
    if (accepted)
        yaw_wave.active = 0;
    __set_PRIMASK(key);
    if (!accepted)
        return TUNING_WRONG_SOURCE;
    return TUNING_OK;
}

uint8_t gimbal_tuning_start_yaw_wave(fp32 amplitude_deg, fp32 frequency_hz, fp32 duration_s)
{
    uint32_t key;
    uint32_t now;
    if (!isfinite(amplitude_deg) || amplitude_deg <= 0.0f || amplitude_deg > 10.0f)
        return TUNING_OUT_OF_RANGE;
    if (!isfinite(frequency_hz) || frequency_hz < 0.1f || frequency_hz > 30.0f)
        return TUNING_OUT_OF_RANGE;
    if (!isfinite(duration_s) || duration_s < 1.0f || duration_s > 60.0f)
        return TUNING_OUT_OF_RANGE;
    key = __get_PRIMASK();
    __disable_irq();
    if (!gimbal_tuning_is_enabled())
    {
        __set_PRIMASK(key);
        return TUNING_NOT_ENABLED;
    }
    if (NX_GetSource() != NX_SOURCE_UART)
    {
        __set_PRIMASK(key);
        return TUNING_WRONG_SOURCE;
    }
    if (!isfinite(YawCtrl.nowAbsoluteAngle))
    {
        __set_PRIMASK(key);
        return TUNING_OUT_OF_RANGE;
    }
    now = HAL_GetTick();
    yaw_wave.center_rad = YawCtrl.nowAbsoluteAngle;
    yaw_wave.amplitude_rad = amplitude_deg * PI / 180.0f;
    yaw_wave.frequency_hz = frequency_hz;
    yaw_wave.duration_ms = (uint32_t)(duration_s * 1000.0f);
    yaw_wave.start_ms = now;
    yaw_wave.keepalive_ms = now;
    yaw_wave.active = 1;
    __set_PRIMASK(key);
    return TUNING_OK;
}

uint8_t gimbal_tuning_keep_yaw_wave(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    if (!yaw_wave.active)
    {
        __set_PRIMASK(key);
        return TUNING_NOT_ENABLED;
    }
    yaw_wave.keepalive_ms = HAL_GetTick();
    __set_PRIMASK(key);
    return TUNING_OK;
}

void gimbal_tuning_stop_yaw_wave(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    yaw_wave.active = 0;
    __set_PRIMASK(key);
}

static void updateYawWave(void)
{
    uint32_t now;
    uint32_t elapsed;
    fp32 phase;
    fp32 angle;
    if (!yaw_wave.active)
        return;
    // 遥控退出、掉线或恢复CAN时立即停止，不能自动重新接管。
    if (!gimbal_tuning_is_enabled() || NX_GetSource() != NX_SOURCE_UART)
    {
        yaw_wave.active = 0;
        return;
    }
    now = HAL_GetTick();
    elapsed = now - yaw_wave.start_ms;
    // 电脑断连后即使控制任务仍在运行，也最多等500 ms就停在最后目标。
    if (now - yaw_wave.keepalive_ms >= 500U || elapsed >= yaw_wave.duration_ms)
    {
        yaw_wave.active = 0;
        return;
    }
    phase = 2.0f * PI * yaw_wave.frequency_hz * ((fp32)elapsed * 0.001f);
    angle = yaw_wave.center_rad + yaw_wave.amplitude_rad * sinf(phase);
    if (angle > PI)
        angle -= 2.0f * PI;
    if (angle < -PI)
        angle += 2.0f * PI;
    YawCtrl.want_AbsoluteAngleCtrl = angle;

    // 波形是已知的正弦，目标的角速度可以直接按公式算，是精确值
    const fp32 omega = 2.0f * PI * yaw_wave.frequency_hz;  // rad/s
    yaw_target_speed = yaw_wave.amplitude_rad * omega * cosf(phase);
    yaw_target_source = YAW_TARGET_SOURCE_WAVE;
}

/**
 * @brief 收到一帧视觉Yaw目标时调用：和上一帧比较，估算目标角速度
 *
 * 角速度 = 两帧目标角之差 / 两帧间隔。24 Hz下估出的是两帧中间时刻的速度，天然晚约半帧。
 * @param targetAngle 这一帧的Yaw目标，rad
 */
static void recordYawFrame(fp32 targetAngle)
{
    const uint32_t now = YawCtrl.nowTime;

    // 目标角无效，不记录，也不估速度
    if (!isfinite(targetAngle))
    {
        return;
    }

    // 第一帧：没有上一帧可比，只记下来，速度按0
    if (yaw_frame_valid == 0)
    {
        yaw_frame_angle = targetAngle;
        yaw_frame_time = now;
        yaw_frame_speed = 0.0f;
        yaw_frame_dt = 0;
        yaw_frame_valid = 1;
        return;
    }

    const uint32_t frameDt = now - yaw_frame_time;

    // 两帧挨得太近（重复帧或成串到达），算出的速度不可信；保留上一帧，等下一帧再比
    if (frameDt < YAW_FF_FRAME_DT_MIN_MS)
    {
        return;
    }

    // 两帧隔得太久（中间断过），速度按0重新开始
    if (frameDt > YAW_FF_FRAME_DT_MAX_MS)
    {
        yaw_frame_angle = targetAngle;
        yaw_frame_time = now;
        yaw_frame_speed = 0.0f;
        yaw_frame_dt = frameDt;
        return;
    }

    // 正常间隔：角度差按最短角度算（跨±180°也对），除以时间得到rad/s
    const fp32 angleChange = radFormat(targetAngle - yaw_frame_angle);
    fp32 speed = angleChange / ((fp32)frameDt * 0.001f);

    // 速度大得不合理，按数据错误处理
    if (fabsf(speed) > YAW_FF_SPEED_MAX)
    {
        speed = 0.0f;
    }

    yaw_frame_angle = targetAngle;
    yaw_frame_time = now;
    yaw_frame_speed = speed;
    yaw_frame_dt = frameDt;
}

/**
 * @brief 每个控制周期给出视觉目标的运动，并按开关做帧间插值
 *
 * 视觉帧没过期时，目标角速度取相邻帧估算值。
 */
static void updateYawVisionMotion(uint8_t automatic)
{
    // 不在自瞄，或还没收到过视觉帧：没有目标运动
    if (automatic == 0)
    {
        return;
    }
    if (yaw_frame_valid == 0)
    {
        return;
    }

    // 视觉帧太久没更新，可能视觉已掉线，不能沿用旧的速度
    const uint32_t sinceFrame = YawCtrl.nowTime - yaw_frame_time;
    if (sinceFrame > FIRE_FRAME_TIMEOUT_MS)
    {
        return;
    }

    yaw_target_speed = yaw_frame_speed;
    yaw_target_source = YAW_TARGET_SOURCE_VISION;
}

/**
 * @brief 检查调试器里填的前馈增益，填错时按0（关闭）处理
 */
static fp32 checkFeedforwardGain(fp32 gain)
{
    // 不是有效数字，按关闭处理
    if (!isfinite(gain))
    {
        return 0.0f;
    }
    // 增益小于0会反着推，按关闭处理
    if (gain < 0.0f)
    {
        return 0.0f;
    }
    // 增益大得不合理，可能多填了一位，按关闭处理
    if (gain > YAW_FF_GAIN_MAX)
    {
        return 0.0f;
    }
    return gain;
}

/**
 * @brief Yaw速度前馈：加到速度环目标上的量
 * @return 内部单位rad/ms；增益为0时为0
 */
static fp32 calcYawSpeedFeedforward(void)
{
    // 对要乘的gain作保护
    const fp32 gain = checkFeedforwardGain(yaw_ff_speed_gain);
    // 目标角速度是rad/s，速度环内部用rad/ms，除以1000
    const fp32 speedFf = gain * yaw_target_speed * 0.001f;
    return speedFf;
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
 * @brief 清空一个轴的静摩擦补偿状态，下次要重新确认停住
 */
static void resetFrictionState(friction_state_t *state)
{
    state->still_valid = 0;
    state->still = 0;
    state->active = 0;
    state->out = 0.0f;
}

/**
 * @brief 从当前时刻、当前角度重新开始静止计时
 */
static void restartStillTimer(const struct GimbalMotorCtrl_s *c, friction_state_t *state)
{
    state->still_start_time = c->nowTime;
    state->still_start_angle = c->nowAbsoluteAngle;
    state->still_valid = 1;
}

/**
 * @brief 判断一个轴是否已经确认停住
 *
 * 电机转速是整数rpm，慢慢蠕动时也会读到0，所以还要看角度漂移：
 * 转速为0、角度离计时起点不超过0.1°，连续满100 ms才算停住。
 * 一旦转动或漂移超出，从当前时刻、当前角度重新计时。
 * 注意转速是电机相对底盘的转速：底盘在转（例如小陀螺）时Yaw不会被判为停住。
 * @return 1表示已确认停住
 */
static uint8_t isAxisStill(const struct GimbalMotorCtrl_s *c, friction_state_t *state)
{
    // 没有转速反馈，无法判断是否停住，按"在动"处理
    if (c->SpeedRpmPoint == NULL)
    {
        state->still_valid = 0;
        return 0;
    }

    // 角度无效，无法判断是否停住，按"在动"处理
    if (!isfinite(c->nowAbsoluteAngle))
    {
        state->still_valid = 0;
        return 0;
    }

    // 刚上电或刚从无力恢复，还没开始计时：从现在开始计时
    if (state->still_valid == 0)
    {
        restartStillTimer(c, state);
        return 0;
    }

    // 电机在转：重新计时
    const int16_t speedRpm = *(c->SpeedRpmPoint);
    if (speedRpm != 0)
    {
        restartStillTimer(c, state);
        return 0;
    }

    // 转速读数为0，但角度在慢慢漂，说明在蠕动：重新计时
    // Yaw会在±180°处跳变，所以先把角度差归一化
    const fp32 drift = fabsf(radFormat(c->nowAbsoluteAngle - state->still_start_angle));
    if (drift > FRICTION_STILL_DRIFT_RAD)
    {
        restartStillTimer(c, state);
        return 0;
    }

    // 静止时间还不够，继续等
    const uint32_t stillTime = c->nowTime - state->still_start_time;
    if (stillTime < FRICTION_STILL_MS)
    {
        return 0;
    }

    return 1;
}

/**
 * @brief 检查调试器里填的门槛，填错时按0（关闭）处理
 */
static fp32 checkFrictionThreshold(fp32 threshold)
{
    // 不是有效数字，按关闭处理
    if (!isfinite(threshold))
    {
        return 0.0f;
    }

    // 门槛是幅值，小于0说明填错了，按关闭处理
    if (threshold < 0.0f)
    {
        return 0.0f;
    }

    // 超过允许的最大值，可能多填了一位，按关闭处理
    if (threshold > FRICTION_MAX_OUT)
    {
        return 0.0f;
    }

    return threshold;
}

/**
 * @brief 计算一个轴本周期的静摩擦补偿量（启动补足），结果同时记在state里
 *
 * 下面几条全部满足才补：
 * 1. 已确认停住，见isAxisStill；
 * 2. 离目标超过死区，目标附近不补，避免在目标点来回顶；
 * 3. 位置环要的速度、速度环PID输出，都指向目标方向；
 * 4. PID输出还没达到门槛。
 * 补的是差额：PID已有600、门槛750，只补150。
 * PID已超过门槛、正在刹车、已经转起来，都不补。补偿不写进积分，只在本周期生效。
 * 正方向指PID内部的正方向，也就是角度增大的方向：Pitch为抬头，Yaw为角度读数增大。
 * @param pidOut 速度环P+I+D，Pitch不含重力补偿
 * @param posThreshold 正方向门槛
 * @param negThreshold 负方向门槛，填正数
 * @param errorBand 死区，单位rad
 * @return 本周期要补的量，PID内部方向；不补时为0
 */
static fp32 calcFrictionFill(const struct GimbalMotorCtrl_s *c, friction_state_t *state, fp32 pidOut,
                             fp32 posThreshold, fp32 negThreshold, fp32 errorBand)
{
    const fp32 posOut = checkFrictionThreshold(posThreshold);
    const fp32 negOut = checkFrictionThreshold(negThreshold);
    // 本周期角度环误差（目标 - 实际），gimbal_PID_calc 已归一化到 [-PI, PI]
    const fp32 angleError = c->agl_pid.error[0];
    // 位置环输出，就是交给速度环的目标速度
    const fp32 wantSpeed = c->agl_pid.out;

    // 先清掉上周期的结果，下面只在真正补偿时再置位
    state->active = 0;
    state->out = 0.0f;

    // 每周期都要更新静止计时，所以放在最前面
    const uint8_t still = isAxisStill(c, state);
    state->still = still;

    // 没有确认停住：正在转或刚停下，交给PID自己处理
    if (still == 0)
    {
        return 0.0f;
    }

    // 死区填错（不是有效数字或小于0），整个补偿按关闭处理
    if (!isfinite(errorBand))
    {
        return 0.0f;
    }
    if (errorBand < 0.0f)
    {
        return 0.0f;
    }

    // 目标在正方向
    if (angleError > errorBand)
    {
        // 位置环要的不是正方向的速度，不补
        if (wantSpeed <= 0.0f)
        {
            return 0.0f;
        }
        // PID没有往正方向推（例如正在刹车），不补
        if (pidOut <= 0.0f)
        {
            return 0.0f;
        }
        // PID已经达到门槛，不用补；门槛为0时也从这里退出
        if (pidOut >= posOut)
        {
            return 0.0f;
        }
        // 只补差额，让PID加补偿正好等于门槛
        state->active = 1;
        state->out = posOut - pidOut;
        return state->out;
    }

    // 目标在负方向
    if (angleError < -errorBand)
    {
        // 位置环要的不是负方向的速度，不补
        if (wantSpeed >= 0.0f)
        {
            return 0.0f;
        }
        // PID没有往负方向推（例如正在刹车），不补
        if (pidOut >= 0.0f)
        {
            return 0.0f;
        }
        // PID已经达到门槛，不用补；门槛为0时也从这里退出
        if (pidOut <= -negOut)
        {
            return 0.0f;
        }
        // 只补差额，让PID加补偿正好等于负的门槛
        state->active = 1;
        state->out = -negOut - pidOut;
        return state->out;
    }

    // 已在目标附近的死区内，不补
    return 0.0f;
}

/**
 * @brief 合并Pitch速度环PID、静摩擦补偿和重力补偿，处理积分及总输出限幅
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

    // PID推不动时补到门槛；补偿不写进积分，只加在本周期总输出上
    const fp32 frictionOut = calcFrictionFill(&PitchCtrl, &pitch_friction_state, pidOut,
                                              pitch_friction_up_out, pitch_friction_down_out,
                                              pitch_friction_error_rad);
    debug->friction_still = pitch_friction_state.still;
    debug->friction_active = pitch_friction_state.active;

    // 总输出使用未限幅的PID值，先加两项补偿再限幅，不能直接用pid->out
    totalOut = pidOut + frictionOut + gravityOut;
    const fp32 totalBeforeLimit = totalOut;
    range_limit_inside(totalOut, pid->max_out);

    // 记录计算结果和各自是否到达上限，调试数据不参与控制计算
    debug->p_out = pid->Pout;
    debug->i_out = pid->Iout;
    debug->d_out = pid->Dout;
    debug->pid_unclamped_out = pidOut;
    debug->friction_out = frictionOut;
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
    // Yaw速度前馈：目标角速度直接加到速度环目标上，位置环只修剩下的偏差
    const fp32 yawSpeedFf = calcYawSpeedFeedforward();
    yaw_speed_setpoint = YawCtrl.agl_pid.out + yawSpeedFf;
    // Yaw轴速度环PID计算
    PID_calc(&(YawCtrl.spd_pid), YawCtrl.radspeed, yaw_speed_setpoint);
    // Yaw静摩擦补偿：PID推不动时补到门槛；补偿不写进积分，只加在本周期输出上
    const fp32 yawFrictionOut = calcFrictionFill(&YawCtrl, &yaw_friction_state, YawCtrl.spd_pid.out,
                                                 yaw_friction_pos_out, yaw_friction_neg_out,
                                                 yaw_friction_error_rad);
    fp32 yawTotalOut = YawCtrl.spd_pid.out + yawFrictionOut;
    // 保险：门槛上限远小于输出上限，正常碰不到，但加完补偿后仍统一限幅一次
    range_limit_inside(yawTotalOut, YAW_VOLT_MAX_OUT);
    YawCtrl.Volt_out = (int16_t)yawTotalOut;

    // 记录前馈，供调试器观察
    yaw_ff_debug.source = yaw_target_source;
    yaw_ff_debug.target_speed_rad_s = yaw_target_speed;
    yaw_ff_debug.speed_ff_rad_s = yawSpeedFf * 1000.0f;
    yaw_ff_debug.frame_dt_ms = yaw_frame_dt;

    // Yaw轴无力：输出清零，恢复后要重新确认停住
    if (YawCtrl.nopowerflag)
    {
        YawCtrl.Volt_out = 0;
        resetFrictionState(&yaw_friction_state);
    }
    // 遥控掉线：主循环会把输出清零，这里让静止计时作废，恢复后重新确认停住
    if (rc_online == 0)
    {
        resetFrictionState(&yaw_friction_state);
    }

    /* 无力或遥控掉线时不积累 Pitch 积分，避免恢复使能后带入停机误差。 */
    if (PitchCtrl.nopowerflag || rc_online == 0)
    {
        PID_clear(&(PitchCtrl.spd_pid));
        PID_clear(&(PitchCtrl.agl_pid));
        pitch_gravity_output = 0.0f;
        resetFrictionState(&pitch_friction_state); // 恢复使能后重新确认静止
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
 * @brief 记录最近一帧视觉数据中与开火有关的部分
 */
static void recordFireFrame(const CAN_Send_to_MCU *frame, uint8_t source)
{
    fire_frame_time = PitchCtrl.nowTime;
    fire_frame_source = source;
    fire_frame_fire = frame->fire;
    fire_frame_rev = (uint16_t)frame->rev; // 按无符号数解析各位
    fire_frame_valid = 1;
}

/**
 * @brief 按距离算出一个轴的允许误差
 * @param halfSize 允许偏离的半宽或半高，单位m
 * @param distance 目标距离，单位m，调用前已检查大于0
 * @return 允许误差，单位rad；参数填错时返回0，相当于不开火
 */
static fp32 calcFireThreshold(fp32 halfSize, fp32 distance)
{
    // 距离较远时角度很小，直接用 半宽/距离 代替反正切，误差可以忽略
    const fp32 threshold = halfSize / distance - fire_spread_rad;

    // 参数填错导致结果不是有效数字，返回0，后面会判定为不开火
    if (!isfinite(threshold))
    {
        return 0.0f;
    }

    // 近处允许误差封顶，避免阈值太宽乱打
    if (threshold > fire_max_rad)
    {
        return fire_max_rad;
    }

    return threshold;
}

/**
 * @brief 检查本周期是否满足开火的全部条件（不含保持时间）
 * @return 1表示全部满足
 */
static uint8_t checkFireWindow(fire_gate_debug_t *debug)
{
    // 不在自瞄、遥控掉线或无力时，不开火
    if (robotIsAuto() == 0)
    {
        return 0;
    }
    if (rc_online == 0)
    {
        return 0;
    }
    if (PitchCtrl.nopowerflag == 1)
    {
        return 0;
    }

    // 还没收到过视觉帧
    if (fire_frame_valid == 0)
    {
        return 0;
    }

    // 串口模拟只用于调云台，不允许开火
    if (fire_frame_source != NX_SOURCE_CAN)
    {
        return 0;
    }

    // 视觉帧太久没更新，可能视觉已掉线，不能沿用旧的开火请求
    const uint32_t frameAge = PitchCtrl.nowTime - fire_frame_time;
    debug->frame_age_ms = frameAge;
    if (frameAge > FIRE_FRAME_TIMEOUT_MS)
    {
        return 0;
    }
    debug->frame_fresh = 1;

    // 视觉没有允许开火（例如没识别到目标）
    if (fire_frame_fire != 1)
    {
        return 0;
    }
    debug->vision_fire = 1;

    // 视觉要求这次只瞄准不发射
    const uint16_t aimOnly = fire_frame_rev & 0x0001;
    if (aimOnly == 1)
    {
        return 0;
    }

    // 取出距离：rev的bit1~15，单位cm；0表示视觉没给距离
    const uint16_t distanceCm = fire_frame_rev >> 1;
    uint8_t distanceValid = 1;

    // 距离太小：包括0（视觉没给距离，例如视觉那边还没改协议），按无效处理
    if (distanceCm < FIRE_DISTANCE_MIN_CM)
    {
        distanceValid = 0;
    }
    // 距离大得不合理，按数据错误处理
    if (distanceCm > FIRE_DISTANCE_MAX_CM)
    {
        distanceValid = 0;
    }

    // 距离有效：用视觉给的距离；无效：按默认3m算允许误差，照常判断能不能开火
    fp32 distance = FIRE_DISTANCE_DEFAULT_M;
    if (distanceValid == 1)
    {
        distance = (fp32)distanceCm * 0.01f;
    }
    else
    {
        debug->distance_fallback = 1;
    }
    debug->distance_m = distance;
    debug->distance_valid = distanceValid;

    // 按距离算两个轴的允许误差
    const fp32 yawThreshold = calcFireThreshold(fire_yaw_half_m, distance);
    const fp32 pitchThreshold = calcFireThreshold(fire_pitch_half_m, distance);
    debug->yaw_threshold_rad = yawThreshold;
    debug->pitch_threshold_rad = pitchThreshold;

    // 下限或上限填错（不是有效数字），不开火
    if (!isfinite(fire_min_rad))
    {
        return 0;
    }
    if (!isfinite(fire_max_rad))
    {
        return 0;
    }
    // 允许误差比下限还小，说明太远打不准，不开火
    if (yawThreshold < fire_min_rad)
    {
        return 0;
    }
    if (pitchThreshold < fire_min_rad)
    {
        return 0;
    }
    debug->threshold_ok = 1;

    // 两个轴离目标角各差多少，Pitch目标已经过限幅
    const fp32 yawError = fabsf(radFormat(YawCtrl.want_AbsoluteAngleCtrl - YawCtrl.nowAbsoluteAngle));
    const fp32 pitchError = fabsf(radFormat(PitchCtrl.want_AbsoluteAngleCtrl - PitchCtrl.nowAbsoluteAngle));
    debug->yaw_error_rad = yawError;
    debug->pitch_error_rad = pitchError;

    // Yaw还没对准
    if (yawError > yawThreshold)
    {
        return 0;
    }
    // Pitch还没对准
    if (pitchError > pitchThreshold)
    {
        return 0;
    }
    debug->aimed = 1;

    return 1;
}

/**
 * @brief 每个控制周期判断一次是否允许开火
 *
 * 条件全部满足、并且连续保持满 fire_hold_ms 才允许开火，
 * 避免云台冲过目标的一瞬间误差经过0时开火。任何一条不满足，计时清零。
 */
static void updateFireGate(void)
{
    fire_gate_debug_t debug = {0};
    const uint8_t inWindow = checkFireWindow(&debug);

    // 本周期条件不满足：计时清零，不允许开火
    if (inWindow == 0)
    {
        fire_hold_valid = 0;
        fire_allowed = 0;
        fire_gate_debug = debug;
        return;
    }

    // 刚满足条件：从现在开始计时
    if (fire_hold_valid == 0)
    {
        fire_hold_start = PitchCtrl.nowTime;
        fire_hold_valid = 1;
    }

    // 保持时间还不够，继续等
    const uint32_t holdTime = PitchCtrl.nowTime - fire_hold_start;
    debug.hold_ms = holdTime;
    if (holdTime < fire_hold_ms)
    {
        fire_allowed = 0;
        fire_gate_debug = debug;
        return;
    }

    fire_allowed = 1;
    debug.allowed = 1;
    fire_gate_debug = debug;
}

/**
 * @brief 供发射任务查询：本周期是否允许开火
 * @return 1表示允许
 */
uint8_t gimbal_fire_allowed(void)
{
    return fire_allowed;
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
