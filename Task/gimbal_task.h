#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H


#include "struct_typedef.h"

/* 重力补偿 = gain * cos(实际Pitch角度 + offset_rad)。
 * 可在调试器或调参串口中修改；上电值见gimbal_task.c中的DEFAULT宏。
 */
typedef struct
{
    fp32 gain;       // 补偿幅值，正负决定PID内部方向；不是安培或力矩值
    fp32 offset_rad; // 补偿曲线的角度偏移，单位rad，允许范围[-PI, PI]
} pitch_gravity_config_t;

/* 本周期的调试记录，仅供查看。
 * 输出量使用GM6020协议控制量刻度；motor_cmd已转换电机方向，
 * 其余输出采用PID内部方向。标志为1表示对应情况发生。
 */
typedef struct
{
    // 目标与反馈
    fp32 requested_angle_rad; // 限幅前的目标角度，单位rad
    fp32 target_angle_rad;    // 限幅后，实际交给位置环的目标角度，单位rad
    fp32 actual_angle_rad;    // 枪口目前角度，弧度
    fp32 speed_rad_s;         // 当前速度，内部rad/ms已换算为rad/s
    fp32 angle_error_rad;     // 离目标差多少角度

    // 速度环PID，I项已经过积分限幅和超限处理
    fp32 p_out;
    fp32 i_out;
    fp32 d_out;
    fp32 pid_unclamped_out;   // P+I+D，尚未做PID总输出限幅

    // 静摩擦补偿、重力补偿与合成输出
    fp32 friction_out;        // 静摩擦补偿补上的差额，不补时为0
    fp32 gravity_target_out;  // 公式计算后、已限幅的目标补偿
    fp32 gravity_out;         // 限制变化速度后，本周期实际采用的补偿
    fp32 total_unclamped_out; // P+I+D+静摩擦补偿+重力补偿，尚未做总输出限幅
    fp32 total_out;           // 总输出限幅后的结果
    int16_t motor_cmd;        // total_out取反并转为整数后的电机指令

    // 各种限幅分开记录，便于判断是哪一步到达上限，标志位，0/1
    uint8_t control_enabled;        // 本周期已执行Pitch控制
    uint8_t target_limited;         // 请求角度被限制在允许范围内
    uint8_t integral_limited;       // I项达到积分上限
    uint8_t pid_saturated;          // P+I+D达到或超过输出上限
    uint8_t total_saturated;        // 加入补偿后达到或超过输出上限
    uint8_t integral_blocked;       // 本次积分加重总输出超限，已撤销
    uint8_t integral_separated;     // 角度误差超过分离阈值，本周期暂停积分
    uint8_t gravity_limited;        // 公式算出的补偿超过补偿自身上限
    uint8_t gravity_config_invalid; // 补偿参数或当前角度无效，已撤掉补偿
    uint8_t friction_still;         // 已确认停住（转速为0且角度不漂，连续满100 ms）
    uint8_t friction_active;        // 本周期静摩擦补偿生效
} pitch_control_debug_t;

extern volatile pitch_gravity_config_t pitch_gravity_config;
extern volatile pitch_control_debug_t pitch_control_debug;
/* 积分分离阈值（rad）：角度误差超过它时暂停积分，可在调试器里修改。 */
extern volatile fp32 pitch_i_separation_rad;
/* 静摩擦补偿：抬头/低头门槛（PID输出不到门槛时补到门槛，0为关闭）、死区（rad）。 */
extern volatile fp32 pitch_friction_up_out;
extern volatile fp32 pitch_friction_down_out;
extern volatile fp32 pitch_friction_error_rad;
/* Yaw静摩擦补偿：角度增大/减小方向的门槛（0为关闭）、死区（rad）。
 * 补偿状态可在调试器里看 yaw_friction_state 的 still / active / out。 */
extern volatile fp32 yaw_friction_pos_out;
extern volatile fp32 yaw_friction_neg_out;
extern volatile fp32 yaw_friction_error_rad;

/* 开火门控的调试记录，每个控制周期刷新，只供观察。标志为1表示该条件已满足。
 * 条件按顺序检查，前面一条不满足时，后面的字段保持为0。 */
typedef struct
{
    uint32_t frame_age_ms;     // 最近一帧视觉数据距现在多久
    fp32 distance_m;           // 本次计算用的距离：视觉给的距离，无效时为默认3m
    fp32 yaw_threshold_rad;    // 按距离算出的Yaw允许误差
    fp32 pitch_threshold_rad;  // 按距离算出的Pitch允许误差
    fp32 yaw_error_rad;        // Yaw离目标角多远（绝对值）
    fp32 pitch_error_rad;      // Pitch离目标角多远（绝对值）
    uint32_t hold_ms;          // 已经连续对准多久
    uint8_t frame_fresh;       // 视觉帧没有过期
    uint8_t vision_fire;       // 视觉允许开火（fire=1）
    uint8_t distance_valid;    // 视觉给的距离有效
    uint8_t distance_fallback; // 视觉给的距离无效，已按默认3m计算允许误差
    uint8_t threshold_ok;      // 允许误差不小于下限（不算太远）
    uint8_t aimed;             // 两个轴都在允许误差内
    uint8_t allowed;           // 最终允许开火
} fire_gate_debug_t;

/* 开火门控参数：允许误差 = 半宽(m)/距离(m) - 散布角(rad)，再限制在[下限, 上限]。
 * 低于下限说明太远，不开火。fire_hold_ms为需要连续对准的时间。 */
extern volatile fp32 fire_yaw_half_m;
extern volatile fp32 fire_pitch_half_m;
extern volatile fp32 fire_spread_rad;
extern volatile fp32 fire_min_rad;
extern volatile fp32 fire_max_rad;
extern volatile uint32_t fire_hold_ms;
extern volatile fire_gate_debug_t fire_gate_debug;
/* 发射任务查询本周期是否允许开火：视觉允许、帧未过期、两轴已对准并保持。 */
uint8_t gimbal_fire_allowed(void);

/* Yaw目标从哪来：前馈用的目标角速度、角加速度的来源。 */
enum yaw_target_source
{
    YAW_TARGET_SOURCE_NONE = 0,   // 手动或没有有效目标，前馈为0
    YAW_TARGET_SOURCE_VISION = 1, // 视觉帧（CAN或串口模拟），速度由相邻帧估算
    YAW_TARGET_SOURCE_WAVE = 2    // C板波形测试，速度和加速度按公式精确计算
};

/* Yaw前馈的调试记录，每个控制周期刷新，只供观察。 */
typedef struct
{
    uint8_t source;            // 见 yaw_target_source
    fp32 target_speed_rad_s;   // 本周期目标角速度
    fp32 target_accel_rad_s2;  // 本周期目标角加速度（视觉来源时为0）
    fp32 speed_ff_rad_s;       // 实际加到速度环目标上的量
    fp32 out_ff;               // 实际加到电机输出上的量
    uint32_t frame_dt_ms;      // 最近两帧视觉的间隔
} yaw_ff_debug_t;

/* Yaw前馈参数：两个增益默认0即关闭；模型参数单位按rad/s、rad/s²。 */
extern volatile fp32 yaw_ff_speed_gain;
extern volatile fp32 yaw_ff_out_gain;
extern volatile fp32 yaw_ff_inertia;
extern volatile fp32 yaw_ff_back_emf;
extern volatile fp32 yaw_ff_friction;
extern volatile uint8_t yaw_ff_interp;
extern volatile yaw_ff_debug_t yaw_ff_debug;

/* Yaw最近一次控制结果，仅供观察，不参与控制计算。 */
typedef struct
{
    fp32 target_angle_rad;
    fp32 actual_angle_rad;
    fp32 target_speed_rad_s; // 速度环目标（位置环输出 + 速度前馈），rad/s
    fp32 speed_rad_s;        // INS角度差分速度，rad/s
    int16_t motor_cmd;       // 实际交给CAN发送的Yaw电机指令
} yaw_control_debug_t;

/* 调参任务读取一致的双轴快照；保留原类型名，不作为控制输入。 */
typedef struct
{
    uint32_t tick_ms;
    pitch_gravity_config_t gravity;
    pitch_control_debug_t control;
    yaw_control_debug_t yaw;
    fire_gate_debug_t fire_gate; // 本周期门控记录，和双轴控制结果一起读取
    fp32 speed_ki;
    uint8_t source;
    uint8_t remote_online;
    uint8_t automatic;
    uint8_t vision_online;
} pitch_tuning_state_t;

typedef struct
{
    fp32 pitch_min_rad;
    fp32 pitch_max_rad;
    fp32 gain_max;
    fp32 offset_max_rad;
    fp32 gravity_max_out;
    fp32 total_max_out;
    fp32 speed_ki_max;
} pitch_tuning_limits_t;

void gimbal_tuning_get_state(pitch_tuning_state_t *state);
void gimbal_tuning_get_limits(pitch_tuning_limits_t *limits);
uint8_t gimbal_tuning_set_gravity(fp32 gain, fp32 offset_rad);
/* 整组检查后原子写入；Ki=0 同时清零 I 项，非零 Ki 保留已有 I 项。 */
uint8_t gimbal_tuning_set_parameters(fp32 gain, fp32 offset_rad, fp32 speed_ki);
uint8_t gimbal_tuning_set_source(uint8_t source);
uint8_t gimbal_tuning_submit_vision(const uint8_t payload[8]);
/* 高频测试由控制任务每2 ms生成目标；串口只负责开始、停止和保活。 */
uint8_t gimbal_tuning_start_yaw_wave(fp32 amplitude_deg, fp32 frequency_hz, fp32 duration_s);
uint8_t gimbal_tuning_keep_yaw_wave(void);
void gimbal_tuning_stop_yaw_wave(void);

void gimbal_task(void const *pvParameters);
extern const struct GimbalMotorCtrl_s* get_wantedyaw_point(void);
extern const struct GimbalMotorCtrl_s* get_wantedpitch_point(void);
fp32 get_yaw_current_angle(void);
fp32 get_yaw_current_speed(void);
fp32 get_pitch_current_angle(void);
fp32 get_pitch_current_speed(void);
#endif
