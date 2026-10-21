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

/* 调参任务读取一致的快照；不会把调试结构体作为控制输入。 */
typedef struct
{
    uint32_t tick_ms;
    pitch_gravity_config_t gravity;
    pitch_control_debug_t control;
    fp32 speed_ki;
    uint8_t source;
    uint8_t remote_online;
    uint8_t automatic;
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

void gimbal_task(void const *pvParameters);
extern const struct GimbalMotorCtrl_s* get_wantedyaw_point(void);
extern const struct GimbalMotorCtrl_s* get_wantedpitch_point(void);
fp32 get_yaw_current_angle(void);
fp32 get_yaw_current_speed(void);
fp32 get_pitch_current_angle(void);
fp32 get_pitch_current_speed(void);
#endif
