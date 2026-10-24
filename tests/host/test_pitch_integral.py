"""验证真实 Pitch 积分、重力前馈及限幅路径；不模拟或宣称实机稳定性。

测试显式设置 Ki=0、重力 gain=0，验证关闭补偿时的基线行为；
积分相关用例在测试里显式设定 TEST_KI，只验证积分路径本身。
"""
from pathlib import Path
import os
import re
import shutil
import subprocess

root = Path(__file__).resolve().parents[2]
out = root / 'build/host-pitch-integral'
out.mkdir(parents=True, exist_ok=True)
cc = os.environ.get('HOST_CC') or shutil.which('gcc')
if not cc:
    raise SystemExit('未找到宿主 GCC；请设置 HOST_CC。')

source = (root / 'Task/gimbal_task.c').read_text(encoding='utf-8-sig')


def function(name):
    # 直接编译工程中的函数体，避免在测试中另写一份控制算法。
    match = re.search(r'^(?:static )?(?:fp32|void|uint8_t) ' + name +
                      r'\([^;]*?\)\s*(?://[^\n]*\n\s*)?\{', source, re.MULTILINE)
    if not match:
        raise AssertionError('找不到函数定义：' + name)
    depth = 1
    end = match.end()
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


(out / 'main.h').write_text('#include <stddef.h>\n', encoding='utf-8')
(out / 'cmsis_compiler.h').write_text('/* 宿主测试不使用 ARM 编译器内建指令。 */\n', encoding='utf-8')
prefix = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "gimbal_task.h"
#include "NX_communicate.h"
#define PI 3.14159265358979323846f
static int test_auto = 1; /* 模拟 robotIsAuto() */
int robotIsAuto(void) { return test_auto; }
typedef struct { float unused; } first_order_filter_type_t;
volatile uint8_t rc_online = 1;
'''
declarations = source[source.index('#define GIMBAL_TASK_CTRL_TIME'):
                      source.index('// 模式相关')]
functions = '\n'.join(function(name) for name in (
    'radFormat', 'gimbal_PID_calc', 'calcPitchGravity',
    'resetFrictionState', 'restartStillTimer', 'isAxisStill', 'checkFrictionThreshold',
    'calcFrictionFill', 'recordYawFrame', 'updateYawVisionMotion', 'checkFeedforwardGain',
    'calcYawSpeedFeedforward', 'calcYawOutputFeedforward',
    'limitPitchOutput', 'pitch_speed_pid_calc', 'unitPID_calc',
    'recordFireFrame', 'calcFireThreshold', 'checkFireWindow', 'updateFireGate',
    'gimbal_fire_allowed'))
tests = r'''
static void near_value_at(float a, float b, int line)
{
    if (fabsf(a - b) >= 0.002f) fprintf(stderr, "near_value failed at generated line %d: %f != %f\n", line, a, b);
    assert(fabsf(a - b) < 0.002f);
}
#define near_value(a, b) near_value_at((a), (b), __LINE__)

#define TEST_KI 8000.0f /* 仅用于验证积分路径，不是工程默认值 */

static int16_t test_pitch_rpm; /* 模拟 Pitch 6020 的整数 rpm 反馈 */
static int16_t test_yaw_rpm;   /* 模拟 Yaw 6020 的整数 rpm 反馈 */

static void setup_default(void)
{
    memset(&PitchCtrl, 0, sizeof(PitchCtrl));
    memset(&YawCtrl, 0, sizeof(YawCtrl));
    memset(&yaw_wave, 0, sizeof(yaw_wave));
    memset((void *)&pitch_control_debug, 0, sizeof(pitch_control_debug));
    pitch_gravity_config.gain = 0;
    pitch_gravity_config.offset_rad = 0;
    pitch_gravity_output = 0;
    /* 基线用例显式关闭摩擦补偿（门槛0），不依赖工程里正在调试的默认门槛；
     * 补偿代码照常运行，用来验证门槛为0时输出与基线逐周期一致。 */
    pitch_friction_up_out = 0;
    pitch_friction_down_out = 0;
    pitch_friction_error_rad = PITCH_FRICTION_ERROR_RAD_DEFAULT;
    yaw_friction_pos_out = 0;
    yaw_friction_neg_out = 0;
    yaw_friction_error_rad = YAW_FRICTION_ERROR_RAD_DEFAULT;
    resetFrictionState(&pitch_friction_state);
    resetFrictionState(&yaw_friction_state);
    /* 前馈显式关闭，目标运动清零。 */
    yaw_ff_speed_gain = 0;
    yaw_ff_out_gain = 0;
    yaw_ff_inertia = YAW_FF_INERTIA_DEFAULT;
    yaw_ff_back_emf = YAW_FF_BACK_EMF_DEFAULT;
    yaw_ff_friction = YAW_FF_FRICTION_DEFAULT;
    yaw_ff_interp = 0;
    yaw_target_speed = 0;
    yaw_target_accel = 0;
    yaw_target_source = YAW_TARGET_SOURCE_NONE;
    yaw_frame_valid = 0;
    test_pitch_rpm = 0;
    test_yaw_rpm = 0;
    PitchCtrl.SpeedRpmPoint = &test_pitch_rpm;
    YawCtrl.SpeedRpmPoint = &test_yaw_rpm;
    rc_online = 1;
    PID_init(&PitchCtrl.spd_pid, PID_POSITION, pitchSpdPIDco,
             PITCH_VOLT_MAX_OUT, PITCH_VOLT_MAX_IOUT);
    PitchCtrl.spd_pid.Ki = 0; /* 基线测试显式关闭积分，不约束用户正在调试的上电默认值。 */
    PID_init(&PitchCtrl.agl_pid, PID_POSITION, pitchAglPIDco,
             PITCH_AGL_SPD_MAX_OUT, PITCH_AGL_SPD_MAX_IOUT);
    PID_init(&YawCtrl.spd_pid, PID_POSITION, yawSpdPIDco,
             YAW_VOLT_MAX_OUT, YAW_VOLT_MAX_IOUT);
    PID_init(&YawCtrl.agl_pid, PID_POSITION, yawAglPIDco,
             YAW_AGL_SPD_MAX_OUT, YAW_AGL_SPD_MAX_IOUT);
    for (int i = 0; i < 2; ++i) {
        gimbalCtrl[i]->lastTime = 0;
        gimbalCtrl[i]->nowTime = 2;
    }
}

static void setup(void)
{
    setup_default();
    PitchCtrl.spd_pid.Ki = TEST_KI;
}

static float integrate_for_ms(uint32_t period_ms, int duration_ms)
{
    setup();
    PitchCtrl.agl_pid.Kd = 0;
    PitchCtrl.want_AbsoluteAngleCtrl = 0.01f;
    PitchCtrl.nowTime = period_ms;
    for (int t = 0; t < duration_ms; t += (int)period_ms)
        unitPID_calc();
    near_value(PitchCtrl.spd_pid.Ki, TEST_KI);
    return PitchCtrl.spd_pid.Iout;
}

/* 按 2 ms 周期推进时间，运行若干毫秒。 */
static void run_ms(int duration_ms)
{
    for (int t = 0; t < duration_ms; t += 2) {
        PitchCtrl.lastTime = PitchCtrl.nowTime;
        PitchCtrl.nowTime += 2;
        YawCtrl.lastTime = YawCtrl.nowTime;
        YawCtrl.nowTime += 2;
        unitPID_calc();
    }
}

/* 摩擦补偿用例：只留速度环 P，目标误差 err 时 PID 输出 = 4e6 * 0.015 * err。 */
static void setup_friction(float error_rad)
{
    setup_default();
    PitchCtrl.agl_pid.Kd = 0;
    PitchCtrl.spd_pid.Kd = 0;
    pitch_friction_up_out = 750;
    pitch_friction_down_out = 1000;
    PitchCtrl.want_AbsoluteAngleCtrl = error_rad;
}

/* 直接调用 Pitch 的补偿计算，用当前门槛和死区。 */
static float pitch_fill(float pid_out)
{
    return calcFrictionFill(&PitchCtrl, &pitch_friction_state, pid_out,
                            pitch_friction_up_out, pitch_friction_down_out,
                            pitch_friction_error_rad);
}

static void friction_tests(void)
{
    /* 必须连续静止满 100 ms 才补；补的是差额：PID 600、门槛 750，只补 150。 */
    setup_friction(0.01f);
    run_ms(98);
    assert(!pitch_control_debug.friction_still && !pitch_control_debug.friction_active);
    near_value(pitch_control_debug.friction_out, 0);
    near_value(pitch_control_debug.total_out, 600);
    run_ms(4);
    assert(pitch_control_debug.friction_still && pitch_control_debug.friction_active);
    near_value(pitch_control_debug.pid_unclamped_out, 600);
    near_value(pitch_control_debug.friction_out, 150);
    near_value(pitch_control_debug.total_out, 750);
    assert(PitchCtrl.Volt_out == -750);

    /* 补偿与重力前馈叠加：门槛只约束 PID 部分，重力照常加上。 */
    setup_friction(0.01f);
    pitch_gravity_config.gain = 300;
    run_ms(200);
    near_value(pitch_control_debug.friction_out, 150);
    near_value(pitch_control_debug.total_out, 750 + 300);

    /* 低头方向用自己的门槛：PID -600、门槛 1000，补 -400。 */
    setup_friction(-0.01f);
    run_ms(200);
    assert(pitch_control_debug.friction_active);
    near_value(pitch_control_debug.friction_out, -400);
    near_value(pitch_control_debug.total_out, -1000);

    /* 目标附近死区内（0.005 rad < 0.0087 rad）停住也不补。 */
    setup_friction(0.005f);
    run_ms(200);
    assert(pitch_control_debug.friction_still && !pitch_control_debug.friction_active);
    near_value(pitch_control_debug.friction_out, 0);
    /* 死区可在运行时改小。 */
    pitch_friction_error_rad = 0.004f;
    run_ms(2);
    assert(pitch_control_debug.friction_active);
    near_value(pitch_control_debug.total_out, 750);

    /* PID 已超过门槛：不补，也不会把输出拉低到门槛。 */
    setup_friction(0.02f);
    run_ms(200);
    assert(!pitch_control_debug.friction_active);
    near_value(pitch_control_debug.total_out, 1200);

    /* PID 与目标方向相反（在刹车）：不补。 */
    setup_friction(0.01f);
    PitchCtrl.radspeed = 0.0004f; /* 速度环误差 0.00015-0.0004，PID = -1000 */
    run_ms(200);
    assert(pitch_control_debug.friction_still && !pitch_control_debug.friction_active);
    near_value(pitch_control_debug.total_out, -1000);

    /* 位置环要的速度方向与误差方向不一致（例如角度环 D 项反向）：不补。 */
    setup_friction(0.01f);
    run_ms(200);
    {
        PitchCtrl.agl_pid.error[0] = 0.01f;
        PitchCtrl.agl_pid.out = 0.0001f;
        near_value(pitch_fill(600), 150);
        PitchCtrl.agl_pid.out = -0.0001f;
        near_value(pitch_fill(600), 0);
        PitchCtrl.agl_pid.error[0] = -0.01f;
        PitchCtrl.agl_pid.out = -0.0001f;
        near_value(pitch_fill(-600), -400);
        near_value(pitch_fill(200), 0); /* 低头时 PID 在往上推（刹车）：不补 */
        PitchCtrl.agl_pid.out = 0.0001f;
        near_value(pitch_fill(-600), 0);
    }

    /* 转起来立即退出补偿；再次停住要重新等满 100 ms。 */
    setup_friction(0.01f);
    run_ms(200);
    assert(pitch_control_debug.friction_active);
    test_pitch_rpm = 1;
    run_ms(2);
    assert(!pitch_control_debug.friction_still && !pitch_control_debug.friction_active);
    near_value(pitch_control_debug.total_out, 600);
    test_pitch_rpm = 0;
    run_ms(98);
    assert(!pitch_control_debug.friction_active);
    run_ms(4);
    assert(pitch_control_debug.friction_active);

    /* rpm 读数为 0 但角度蠕动超过 0.1°：视为在动，重新计时。 */
    setup_friction(0.01f);
    run_ms(200);
    PitchCtrl.nowAbsoluteAngle = 0.002f;
    PitchCtrl.want_AbsoluteAngleCtrl = 0.012f; /* 误差不变，只验证静止判断 */
    run_ms(2);
    assert(!pitch_control_debug.friction_active);
    run_ms(98);
    assert(!pitch_control_debug.friction_active);
    run_ms(4);
    assert(pitch_control_debug.friction_active);
    /* 漂移不超过 0.1° 时不重新计时。 */
    PitchCtrl.nowAbsoluteAngle = 0.0035f;
    PitchCtrl.want_AbsoluteAngleCtrl = 0.0135f;
    run_ms(2);
    assert(pitch_control_debug.friction_active);

    /* 补偿不写进积分：同样的过程，开和关补偿积分完全相同。 */
    float integral_with[2];
    for (int on = 0; on <= 1; ++on) {
        setup_friction(0.01f);
        PitchCtrl.spd_pid.Ki = TEST_KI;
        PitchCtrl.spd_pid.max_iout = 10;  /* 积分很小，保持 PID 在门槛以下 */
        if (!on) pitch_friction_up_out = 0;
        run_ms(300);
        assert(pitch_control_debug.friction_active == on);
        integral_with[on] = PitchCtrl.spd_pid.Iout;
    }
    near_value(integral_with[0], integral_with[1]);

    /* 门槛或死区填错（NaN、负数、超过上限）都按关闭处理。 */
    const float bad_values[] = {NAN, INFINITY, -750, FRICTION_MAX_OUT + 1};
    for (int i = 0; i < 4; ++i) {
        setup_friction(0.01f);
        pitch_friction_up_out = bad_values[i];
        run_ms(200);
        assert(!pitch_control_debug.friction_active);
        near_value(pitch_control_debug.total_out, 600);
    }
    setup_friction(0.01f);
    pitch_friction_up_out = FRICTION_MAX_OUT;
    run_ms(200);
    near_value(pitch_control_debug.total_out, FRICTION_MAX_OUT);
    for (int i = 0; i < 3; ++i) {
        setup_friction(0.01f);
        pitch_friction_error_rad = bad_values[i];
        run_ms(200);
        assert(!pitch_control_debug.friction_active);
    }

    /* 没有转速反馈指针：无法确认停住，不补。 */
    setup_friction(0.01f);
    PitchCtrl.SpeedRpmPoint = NULL;
    run_ms(200);
    assert(!pitch_control_debug.friction_still && !pitch_control_debug.friction_active);

    /* 无力后恢复：静止计时清零，要重新等满 100 ms。 */
    setup_friction(0.01f);
    run_ms(200);
    assert(pitch_control_debug.friction_active);
    PitchCtrl.nopowerflag = 1;
    run_ms(2);
    near_value(pitch_control_debug.friction_out, 0);
    assert(PitchCtrl.Volt_out == 0);
    PitchCtrl.nopowerflag = 0;
    run_ms(98);
    assert(!pitch_control_debug.friction_active);
    run_ms(4);
    assert(pitch_control_debug.friction_active);
}

/* Yaw 摩擦补偿用例：Yaw 只有 P，误差 err 时 PID 输出 = 1e7 * 0.012 * err。
 * Pitch 保持目标=实际，不产生补偿，只看 Yaw。 */
static void setup_yaw_friction(float error_rad)
{
    setup_default();
    yaw_friction_pos_out = 1500;
    yaw_friction_neg_out = 2000;
    /* 死区单独设小：Kp 较大时，默认死区边缘的 PID 输出就已超过门槛，用例会测不到补偿。 */
    yaw_friction_error_rad = 0.002f;
    YawCtrl.want_AbsoluteAngleCtrl = error_rad;
}

static void yaw_friction_tests(void)
{
    const float yaw_gain = YAW_SPD_KP * YAW_AGL_KP; /* 每 rad 误差对应的输出 */
    /* 误差按当前 Kp 反推，让 PID 输出正好是 1000，低于两个门槛；改 Kp 不影响用例。 */
    const float e1 = 1000.0f / yaw_gain;

    /* 门槛为0时与原来的 Yaw 输出完全一致（由 setup_default 的基线用例覆盖），这里再看一次停住后。 */
    setup_default();
    YawCtrl.want_AbsoluteAngleCtrl = e1;
    run_ms(200);
    assert(!yaw_friction_state.active);
    assert(YawCtrl.Volt_out == (int16_t)(yaw_gain * e1));

    /* 正方向：停住满 100 ms 后补到门槛 1500；Pitch 的门槛不影响 Yaw。 */
    setup_yaw_friction(e1);
    pitch_friction_up_out = 4000;
    run_ms(98);
    assert(!yaw_friction_state.still && !yaw_friction_state.active);
    assert(YawCtrl.Volt_out == (int16_t)(yaw_gain * e1));
    run_ms(4);
    assert(yaw_friction_state.still && yaw_friction_state.active);
    near_value(yaw_friction_state.out, 1500 - yaw_gain * e1);
    assert(YawCtrl.Volt_out == 1500);
    assert(!pitch_control_debug.friction_active);

    /* 负方向用自己的门槛 2000。 */
    setup_yaw_friction(-e1);
    run_ms(200);
    assert(YawCtrl.Volt_out == -2000);

    /* 死区内不补；PID 已超过门槛不补。 */
    setup_yaw_friction(0.001f) /* 小于死区 0.002 */;
    run_ms(200);
    assert(yaw_friction_state.still && !yaw_friction_state.active);
    setup_yaw_friction(2000.0f / yaw_gain) /* PID 2000，超过门槛 1500 */;
    run_ms(200);
    assert(!yaw_friction_state.active);
    assert(YawCtrl.Volt_out == (int16_t)(yaw_gain * (2000.0f / yaw_gain)));

    /* 电机相对底盘在转（例如小陀螺时底盘在下面转）：不算停住，不补。 */
    setup_yaw_friction(e1);
    test_yaw_rpm = 3;
    run_ms(200);
    assert(!yaw_friction_state.still && !yaw_friction_state.active);

    /* 角度跨过 ±180°：差值按最短角度算，不会被误判为在动。 */
    setup_yaw_friction(0);
    YawCtrl.nowAbsoluteAngle = PI - 0.0005f;
    YawCtrl.want_AbsoluteAngleCtrl = PI - 0.0005f - e1;
    run_ms(60);
    YawCtrl.nowAbsoluteAngle = -PI + 0.0005f;
    YawCtrl.want_AbsoluteAngleCtrl = -PI + 0.0005f - e1;
    run_ms(60);
    assert(yaw_friction_state.still && yaw_friction_state.active);

    /* 无力或遥控掉线：输出为0，恢复后要重新等满 100 ms。 */
    for (int which = 0; which < 2; ++which) {
        setup_yaw_friction(e1);
        run_ms(200);
        assert(yaw_friction_state.active);
        if (which == 0) YawCtrl.nopowerflag = 1;
        if (which == 1) rc_online = 0;
        run_ms(2);
        if (which == 0) assert(YawCtrl.Volt_out == 0);
        assert(!yaw_friction_state.active);
        YawCtrl.nopowerflag = 0;
        rc_online = 1;
        run_ms(98);
        assert(!yaw_friction_state.active);
        run_ms(4);
        assert(yaw_friction_state.active);
    }

    /* Yaw 门槛填错按关闭处理。 */
    setup_yaw_friction(e1);
    yaw_friction_pos_out = NAN;
    run_ms(200);
    assert(!yaw_friction_state.active);
    assert(YawCtrl.Volt_out == (int16_t)(yaw_gain * e1));
}

/* 开火门控用例：自瞄、两轴误差为0、默认参数。 */
static void setup_fire(void)
{
    setup_default();
    test_auto = 1;
    /* 这些用例按下文明确的47/43 mm测试阈值公式，不依赖实车调试默认半宽。 */
    fire_yaw_half_m = 0.047f;
    fire_pitch_half_m = 0.043f;
    fire_spread_rad = 0; /* 显式设0，不依赖工程里正在调的散布角；散布角单独有用例 */
    fire_min_rad = FIRE_MIN_RAD_DEFAULT;
    fire_max_rad = FIRE_MAX_RAD_DEFAULT;
    fire_hold_ms = FIRE_HOLD_MS_DEFAULT;
    fire_frame_valid = 0;
    fire_hold_valid = 0;
    fire_allowed = 0;
    PitchCtrl.nowTime = 1000;
}

/* 模拟视觉发来一帧：fire 字节、只瞄准位、距离（cm）。 */
static void send_frame(uint8_t fire, uint16_t aim_only, uint16_t distance_cm, uint8_t source)
{
    CAN_Send_to_MCU frame = {0};
    frame.fire = fire;
    frame.rev = (int16_t)(uint16_t)((distance_cm << 1) | aim_only);
    recordFireFrame(&frame, source);
}

/* 推进 n 个 2 ms 控制周期，每周期判断一次。 */
static void fire_cycles(int n)
{
    for (int i = 0; i < n; ++i) {
        PitchCtrl.nowTime += 2;
        updateFireGate();
    }
}

/* 发一帧后跑足够多周期，返回是否允许开火。 */
static uint8_t fire_after_frame(uint8_t fire, uint16_t aim_only, uint16_t distance_cm)
{
    send_frame(fire, aim_only, distance_cm, NX_SOURCE_CAN);
    fire_cycles(10);
    return gimbal_fire_allowed();
}

static void fire_gate_tests(void)
{
    /* 正常：3 m、两轴已对准，连续满 10 ms 才允许；阈值按 半宽/距离 计算。 */
    setup_fire();
    send_frame(1, 0, 300, NX_SOURCE_CAN);
    fire_cycles(5); /* 已保持 8 ms */
    assert(!gimbal_fire_allowed() && fire_gate_debug.aimed && !fire_gate_debug.allowed);
    fire_cycles(1); /* 已保持 10 ms */
    assert(gimbal_fire_allowed() && fire_gate_debug.allowed);
    near_value(fire_gate_debug.distance_m, 3.0f);
    near_value(fire_gate_debug.yaw_threshold_rad, 0.047f / 3.0f);
    near_value(fire_gate_debug.pitch_threshold_rad, 0.043f / 3.0f);

    /* 任一轴误差超过自己的阈值都不允许；误差略小于阈值允许。 */
    const float yaw_th = 0.047f / 3.0f;
    const float pitch_th = 0.043f / 3.0f;
    setup_fire();
    YawCtrl.want_AbsoluteAngleCtrl = yaw_th + 0.0002f;
    assert(!fire_after_frame(1, 0, 300) && !fire_gate_debug.aimed);
    YawCtrl.want_AbsoluteAngleCtrl = -(yaw_th - 0.0002f);
    assert(fire_after_frame(1, 0, 300));
    YawCtrl.want_AbsoluteAngleCtrl = 0;
    PitchCtrl.want_AbsoluteAngleCtrl = pitch_th + 0.0002f;
    assert(!fire_after_frame(1, 0, 300));
    PitchCtrl.want_AbsoluteAngleCtrl = pitch_th - 0.0002f;
    assert(fire_after_frame(1, 0, 300));

    /* Yaw 跨 ±π 时按最短角度算误差。 */
    setup_fire();
    YawCtrl.want_AbsoluteAngleCtrl = 3.14f;
    YawCtrl.nowAbsoluteAngle = -3.14f;
    assert(fire_after_frame(1, 0, 300));
    near_value(fire_gate_debug.yaw_error_rad, 2 * PI - 6.28f);

    /* 远处：允许误差低于下限 0.3° 就不开火。8 m 两轴都够，9 m Pitch 不够，10 m 都不够。 */
    setup_fire();
    assert(fire_after_frame(1, 0, 800));
    assert(!fire_after_frame(1, 0, 900) && fire_gate_debug.distance_valid && !fire_gate_debug.threshold_ok);
    assert(!fire_after_frame(1, 0, 1000));

    /* 近处：允许误差封顶 3°。 */
    setup_fire();
    assert(fire_after_frame(1, 0, 50));
    near_value(fire_gate_debug.yaw_threshold_rad, FIRE_MAX_RAD_DEFAULT);
    near_value(fire_gate_debug.pitch_threshold_rad, FIRE_MAX_RAD_DEFAULT);

    /* 散布角从阈值里减掉。 */
    setup_fire();
    fire_spread_rad = 0.002f;
    assert(fire_after_frame(1, 0, 300));
    near_value(fire_gate_debug.yaw_threshold_rad, 0.047f / 3.0f - 0.002f);

    /* 视觉不允许开火、只瞄准不发射：不开火。 */
    setup_fire();
    assert(!fire_after_frame(0, 0, 300) && fire_gate_debug.frame_fresh && !fire_gate_debug.vision_fire);
    assert(!fire_after_frame(1, 1, 300));
    /* 15 m 是有效距离，但太远，低于下限，不开火（不会被当成无效改按3m）。 */
    assert(fire_after_frame(1, 0, FIRE_DISTANCE_MAX_CM) == 0);
    assert(fire_gate_debug.distance_valid && !fire_gate_debug.distance_fallback);

    /* 距离无效（0、小于30cm、大于15m、最高位为1）：按默认 3 m 的允许误差判断，照常可以开火。 */
    const uint16_t bad_distances[] = {0, FIRE_DISTANCE_MIN_CM - 1, FIRE_DISTANCE_MAX_CM + 1, 0x7FFF};
    for (int i = 0; i < 4; ++i) {
        setup_fire();
        assert(fire_after_frame(1, 0, bad_distances[i]));
        assert(!fire_gate_debug.distance_valid && fire_gate_debug.distance_fallback);
        near_value(fire_gate_debug.distance_m, FIRE_DISTANCE_DEFAULT_M);
        near_value(fire_gate_debug.yaw_threshold_rad, 0.047f / 3.0f);
        near_value(fire_gate_debug.pitch_threshold_rad, 0.043f / 3.0f);
    }
    /* 按默认 3 m 时，误差超过 3 m 的允许误差就不开火。 */
    setup_fire();
    YawCtrl.want_AbsoluteAngleCtrl = 0.047f / 3.0f + 0.0002f;
    assert(!fire_after_frame(1, 0, 0) && fire_gate_debug.distance_fallback && !fire_gate_debug.aimed);
    /* 最小有效距离 30 cm 用视觉给的距离（近处阈值封顶 3°）。 */
    setup_fire();
    assert(fire_after_frame(1, 0, FIRE_DISTANCE_MIN_CM));
    assert(fire_gate_debug.distance_valid && !fire_gate_debug.distance_fallback);
    near_value(fire_gate_debug.distance_m, 0.3f);
    near_value(fire_gate_debug.yaw_threshold_rad, FIRE_MAX_RAD_DEFAULT);

    /* 串口模拟来源不开火。 */
    setup_fire();
    send_frame(1, 0, 300, NX_SOURCE_UART);
    fire_cycles(10);
    assert(!gimbal_fire_allowed());

    /* 没收到过视觉帧不开火（即使帧缓存里残留着看起来有效的数据）。 */
    setup_fire();
    send_frame(1, 0, 300, NX_SOURCE_CAN);
    fire_frame_valid = 0;
    fire_cycles(10);
    assert(!gimbal_fire_allowed());

    /* 视觉帧超过 100 ms 没更新：不能沿用旧的 fire=1。 */
    setup_fire();
    assert(fire_after_frame(1, 0, 300)); /* 帧已过去 20 ms */
    fire_cycles(40); /* 100 ms */
    assert(gimbal_fire_allowed());
    fire_cycles(1);  /* 102 ms */
    assert(!gimbal_fire_allowed() && !fire_gate_debug.frame_fresh);

    /* 退出自瞄、遥控掉线、无力：不开火，恢复后要重新保持满 10 ms。 */
    for (int which = 0; which < 3; ++which) {
        setup_fire();
        assert(fire_after_frame(1, 0, 300));
        if (which == 0) test_auto = 0;
        if (which == 1) rc_online = 0;
        if (which == 2) PitchCtrl.nopowerflag = 1;
        fire_cycles(1);
        assert(!gimbal_fire_allowed());
        test_auto = 1;
        rc_online = 1;
        PitchCtrl.nopowerflag = 0;
        send_frame(1, 0, 300, NX_SOURCE_CAN);
        fire_cycles(5);
        assert(!gimbal_fire_allowed());
        fire_cycles(1);
        assert(gimbal_fire_allowed());
    }

    /* 误差短暂超出一个周期：保持计时清零。 */
    setup_fire();
    assert(fire_after_frame(1, 0, 300));
    YawCtrl.want_AbsoluteAngleCtrl = 0.1f;
    fire_cycles(1);
    assert(!gimbal_fire_allowed());
    YawCtrl.want_AbsoluteAngleCtrl = 0;
    fire_cycles(5);
    assert(!gimbal_fire_allowed());
    fire_cycles(1);
    assert(gimbal_fire_allowed());

    /* 保持时间设为 0：第一次满足条件就允许。 */
    setup_fire();
    fire_hold_ms = 0;
    send_frame(1, 0, 300, NX_SOURCE_CAN);
    fire_cycles(1);
    assert(gimbal_fire_allowed());

    /* 参数填错（非数）一律不开火。 */
    setup_fire();
    fire_yaw_half_m = NAN;
    assert(!fire_after_frame(1, 0, 300));
    setup_fire();
    fire_spread_rad = NAN;
    assert(!fire_after_frame(1, 0, 300));
    setup_fire();
    fire_min_rad = NAN;
    assert(!fire_after_frame(1, 0, 300));
    setup_fire();
    fire_max_rad = NAN;
    assert(!fire_after_frame(1, 0, 300));
}

/* Yaw 前馈用例：Yaw 目标=实际，位置环输出为0，只看前馈本身。 */
static void yaw_ff_tests(void)
{
    /* 增益为0时，即使有目标运动，输出与原来完全一样。 */
    setup_default();
    yaw_target_speed = 1.0f;
    yaw_target_accel = 5.0f;
    unitPID_calc();
    assert(YawCtrl.Volt_out == 0);
    near_value(yaw_ff_debug.out_ff, 0);

    /* 速度前馈：1 rad/s 的目标速度加到速度环目标上（内部 0.001 rad/ms）。 */
    setup_default();
    yaw_ff_speed_gain = 1.0f;
    yaw_target_speed = 1.0f;
    unitPID_calc();
    near_value(yaw_speed_setpoint * 1000, 1.0f);
    near_value(yaw_ff_debug.speed_ff_rad_s, 1.0f);
    assert(YawCtrl.Volt_out == (int16_t)(YAW_SPD_KP * 0.001f));
    /* 增益0.5只加一半。 */
    yaw_ff_speed_gain = 0.5f;
    unitPID_calc();
    near_value(yaw_speed_setpoint * 1000, 0.5f);

    /* 增益填错（非数、负数、大于2）按关闭处理。 */
    const float bad_gains[] = {NAN, -1.0f, YAW_FF_GAIN_MAX + 0.1f};
    for (int i = 0; i < 3; ++i) {
        setup_default();
        yaw_target_speed = 1.0f;
        yaw_target_accel = 5.0f;
        yaw_ff_speed_gain = bad_gains[i];
        yaw_ff_out_gain = bad_gains[i];
        unitPID_calc();
        near_value(yaw_speed_setpoint, 0);
        assert(YawCtrl.Volt_out == 0);
    }

    /* 输出前馈：200×2 + 1100×0.5 + 900 = 1850。 */
    setup_default();
    yaw_ff_out_gain = 1.0f;
    yaw_target_speed = 0.5f;
    yaw_target_accel = 2.0f;
    unitPID_calc();
    near_value(yaw_ff_debug.out_ff, 1850);
    assert(YawCtrl.Volt_out == 1850);
    /* 反方向摩擦项取负。 */
    yaw_target_speed = -0.5f;
    yaw_target_accel = -2.0f;
    unitPID_calc();
    near_value(yaw_ff_debug.out_ff, -1850);
    /* 目标比 5°/s 慢：不加摩擦项。 */
    yaw_target_speed = 0.05f;
    yaw_target_accel = 0;
    unitPID_calc();
    near_value(yaw_ff_debug.out_ff, 1100 * 0.05f);
    /* 输出前馈自身限幅。 */
    yaw_target_speed = 0;
    yaw_target_accel = 1000.0f;
    unitPID_calc();
    near_value(yaw_ff_debug.out_ff, YAW_FF_OUT_MAX);
    /* 模型参数填错：整个输出前馈关闭。 */
    yaw_target_speed = 0.5f;
    yaw_target_accel = 2.0f;
    yaw_ff_friction = NAN;
    unitPID_calc();
    near_value(yaw_ff_debug.out_ff, 0);
    yaw_ff_friction = YAW_FF_FRICTION_DEFAULT;
    yaw_ff_inertia = INFINITY;
    unitPID_calc();
    near_value(yaw_ff_debug.out_ff, 0);

    /* 无力时前馈也不能让电机出力。 */
    setup_default();
    yaw_ff_out_gain = 1.0f;
    yaw_target_speed = 0.5f;
    YawCtrl.nopowerflag = 1;
    unitPID_calc();
    assert(YawCtrl.Volt_out == 0);

    /* 相邻帧估算目标角速度：第一帧速度为0，第二帧 42 ms 后 +0.01 rad。 */
    setup_default();
    YawCtrl.nowTime = 1000;
    recordYawFrame(0.1f);
    near_value(yaw_frame_speed, 0);
    YawCtrl.nowTime = 1042;
    recordYawFrame(0.11f);
    near_value(yaw_frame_speed, 0.01f / 0.042f);
    assert(yaw_frame_dt == 42);
    /* 间隔小于 10 ms 的帧不拿来估速度，保留上一帧。 */
    YawCtrl.nowTime = 1046;
    recordYawFrame(0.5f);
    near_value(yaw_frame_speed, 0.01f / 0.042f);
    near_value(yaw_frame_angle, 0.11f);
    /* 间隔超过 100 ms：速度按0重新开始。 */
    YawCtrl.nowTime = 1200;
    recordYawFrame(0.2f);
    near_value(yaw_frame_speed, 0);
    /* 跨 ±180°：按最短角度算。 */
    YawCtrl.nowTime = 1300;
    recordYawFrame(PI - 0.005f);
    YawCtrl.nowTime = 1340;
    recordYawFrame(-PI + 0.005f);
    near_value(yaw_frame_speed, 0.01f / 0.04f);
    /* 速度大得不合理：按0。 */
    YawCtrl.nowTime = 1380;
    recordYawFrame(-PI + 0.005f + 0.5f);
    near_value(yaw_frame_speed, 0);
    /* 无效角度不记录。 */
    YawCtrl.nowTime = 1420;
    recordYawFrame(NAN);
    near_value(yaw_frame_angle, -PI + 0.005f + 0.5f);

    /* 每周期给出视觉目标运动；插值关闭时目标不动。 */
    setup_default();
    YawCtrl.nowTime = 2000;
    recordYawFrame(0.1f);
    YawCtrl.nowTime = 2040;
    recordYawFrame(0.12f);                 /* 速度 0.5 rad/s */
    YawCtrl.want_AbsoluteAngleCtrl = 0.12f;
    YawCtrl.nowTime = 2060;
    updateYawVisionMotion(1);
    assert(yaw_target_source == YAW_TARGET_SOURCE_VISION);
    near_value(yaw_target_speed, 0.5f);
    near_value(yaw_target_accel, 0);
    near_value(YawCtrl.want_AbsoluteAngleCtrl, 0.12f);
    /* 插值打开：20 ms 后目标往前推 0.5 × 0.02 = 0.01 rad。 */
    yaw_ff_interp = 1;
    updateYawVisionMotion(1);
    near_value(YawCtrl.want_AbsoluteAngleCtrl, 0.13f);
    /* 最多往前推 50 ms。 */
    YawCtrl.nowTime = 2120;
    updateYawVisionMotion(1);
    near_value(YawCtrl.want_AbsoluteAngleCtrl, 0.12f + 0.5f * 0.05f);
    /* 帧超过 100 ms 没更新：不给目标运动，也不再推。 */
    yaw_target_source = YAW_TARGET_SOURCE_NONE;
    yaw_target_speed = 0;
    YawCtrl.want_AbsoluteAngleCtrl = 0.12f;
    YawCtrl.nowTime = 2141;
    updateYawVisionMotion(1);
    assert(yaw_target_source == YAW_TARGET_SOURCE_NONE);
    near_value(yaw_target_speed, 0);
    near_value(YawCtrl.want_AbsoluteAngleCtrl, 0.12f);
    /* 不在自瞄：不给目标运动。 */
    YawCtrl.nowTime = 2060;
    updateYawVisionMotion(0);
    assert(yaw_target_source == YAW_TARGET_SOURCE_NONE);
    yaw_ff_interp = 0;
}

int main(void)
{
    /* Test setup：Ki=0、gain=0，Pitch 输出与基线 gimbal_PID_calc + PID_calc 逐周期一致。 */
    setup_default();
    {
        pid_type_def base_angle = PitchCtrl.agl_pid;
        pid_type_def base_speed = PitchCtrl.spd_pid;
        for (int i = 0; i < 400; ++i) {
            PitchCtrl.nowAbsoluteAngle = 0.3f * sinf(i * 0.02f);
            PitchCtrl.want_AbsoluteAngleCtrl = (i / 50) % 2 ? 0.25f : -0.2f;
            PitchCtrl.radspeed = 0.0004f * cosf(i * 0.05f);
            PitchCtrl.lastTime = PitchCtrl.nowTime;
            PitchCtrl.nowTime += 2;
            gimbal_PID_calc(&base_angle, PitchCtrl.nowAbsoluteAngle, PitchCtrl.want_AbsoluteAngleCtrl);
            PID_calc(&base_speed, PitchCtrl.radspeed, base_angle.out);
            unitPID_calc();
            assert(PitchCtrl.Volt_out == (int16_t)(-base_speed.out));
            near_value(PitchCtrl.spd_pid.Iout, 0);
            near_value(pitch_control_debug.gravity_out, 0);
        }
    }

    /* 持续静态误差时补足输出，相同经过时间不因控制调用次数不同而改变积分。 */
    near_value(integrate_for_ms(1, 120), 72.0f);
    near_value(integrate_for_ms(2, 120), 72.0f);
    near_value(integrate_for_ms(3, 120), 72.0f);
    setup();
    PitchCtrl.want_AbsoluteAngleCtrl = 0.01f;
    unitPID_calc();
    float first_output = -PitchCtrl.Volt_out;
    for (int i = 0; i < 500; ++i) unitPID_calc();
    assert(PitchCtrl.spd_pid.Iout > 500);
    assert(-PitchCtrl.Volt_out > first_output);

    /* 对准后保留抗负载积分；出现反向误差时允许积分自然回退。 */
    PitchCtrl.agl_pid.Kd = PitchCtrl.spd_pid.Kd = 0;
    PitchCtrl.want_AbsoluteAngleCtrl = PitchCtrl.nowAbsoluteAngle = 0;
    float held = PitchCtrl.spd_pid.Iout;
    unitPID_calc();
    near_value(PitchCtrl.spd_pid.Iout, held);
    PitchCtrl.want_AbsoluteAngleCtrl = -0.01f;
    unitPID_calc();
    assert(PitchCtrl.spd_pid.Iout < held);

    /* 积分限幅与总输出限幅在正反两个方向均生效。 */
    for (int sign = -1; sign <= 1; sign += 2) {
        setup();
        PitchCtrl.want_AbsoluteAngleCtrl = sign * 0.01f;
        for (int i = 0; i < 4000; ++i) unitPID_calc();
        near_value(PitchCtrl.spd_pid.Iout, sign * PITCH_VOLT_MAX_IOUT);
        setup();
        PitchCtrl.nowAbsoluteAngle = -sign * 0.3f;
        PitchCtrl.want_AbsoluteAngleCtrl = sign * 0.4f;
        for (int i = 0; i < 100; ++i) unitPID_calc();
        near_value(PitchCtrl.spd_pid.Iout, 0);
        assert(fabsf(PitchCtrl.spd_pid.out) <= PITCH_VOLT_MAX_OUT);
        assert(PitchCtrl.Volt_out == -sign * (int)PITCH_VOLT_MAX_OUT);
    }
    setup();
    PitchCtrl.agl_pid.Kd = PitchCtrl.spd_pid.Kd = 0;
    PitchCtrl.spd_pid.Iout = 2000;
    PitchCtrl.spd_pid.max_out = 1000;
    PitchCtrl.want_AbsoluteAngleCtrl = -0.001f;
    unitPID_calc();
    assert(PitchCtrl.spd_pid.Iout < 2000); /* 饱和时仍允许反向释放。 */

    /* 无力或遥控掉线时双环状态清零，重新使能不带入停机积分。 */
    for (int offline = 0; offline <= 1; ++offline) {
        setup();
        PitchCtrl.spd_pid.Iout = 2000;
        PitchCtrl.agl_pid.Iout = 1;
        PitchCtrl.want_AbsoluteAngleCtrl = 0.2f;
        PitchCtrl.nopowerflag = !offline;
        rc_online = !offline;
        for (int i = 0; i < 100; ++i) unitPID_calc();
        near_value(PitchCtrl.spd_pid.Iout, 0);
        near_value(PitchCtrl.agl_pid.Iout, 0);
        assert(PitchCtrl.Volt_out == 0);
        PitchCtrl.nopowerflag = 0;
        rc_online = 1;
        PitchCtrl.want_AbsoluteAngleCtrl = 0;
        unitPID_calc();
        assert(PitchCtrl.Volt_out == 0);
    }

    /* 启动长间隔不补积分，正常 tick 回绕仍计算正确时间差。 */
    setup();
    PitchCtrl.agl_pid.Kd = 0;
    PitchCtrl.want_AbsoluteAngleCtrl = 0.01f;
    PitchCtrl.nowTime = 500;
    unitPID_calc();
    near_value(PitchCtrl.spd_pid.Iout, 0);
    PitchCtrl.nowTime = PitchCtrl.lastTime = 3;
    unitPID_calc();
    near_value(PitchCtrl.spd_pid.Iout, 0);
    PitchCtrl.lastTime = UINT32_MAX - 1;
    PitchCtrl.nowTime = 1;
    unitPID_calc();
    near_value(PitchCtrl.spd_pid.Iout, 1.8f);

    /* Yaw 仍与原有 gimbal_PID_calc + PID_calc 结果一致。 */
    setup();
    pid_type_def yaw_angle = YawCtrl.agl_pid;
    pid_type_def yaw_speed = YawCtrl.spd_pid;
    for (int i = 0; i < 100; ++i) {
        YawCtrl.nowAbsoluteAngle = i * 0.001f;
        YawCtrl.want_AbsoluteAngleCtrl = 0.2f - i * 0.0005f;
        YawCtrl.radspeed = (i % 3) * 0.0001f;
        gimbal_PID_calc(&yaw_angle, YawCtrl.nowAbsoluteAngle, YawCtrl.want_AbsoluteAngleCtrl);
        PID_calc(&yaw_speed, YawCtrl.radspeed, yaw_angle.out);
        unitPID_calc();
        near_value(YawCtrl.spd_pid.out, yaw_speed.out);
        assert(YawCtrl.Volt_out == (int16_t)yaw_speed.out);
    }
    /* 零误差也有保持补偿；电机方向只在最终输出处取反。 */
    setup();
    pitch_gravity_config.gain = -1200;
    for (int i = 0; i < 100; ++i) unitPID_calc();
    near_value(pitch_control_debug.gravity_out, -1200);
    near_value(PitchCtrl.spd_pid.out, 0);
    assert(PitchCtrl.Volt_out == 1200);
    assert(pitch_control_debug.motor_cmd == PitchCtrl.Volt_out);
    assert(pitch_control_debug.control_enabled);
    assert(!pitch_control_debug.total_saturated);

    /* 补偿随实际角度变化，保持、上行和下行均不按误差方向翻转。 */
    setup();
    pitch_gravity_config.gain = 1200;
    PitchCtrl.nowAbsoluteAngle = PI / 6;
    PitchCtrl.want_AbsoluteAngleCtrl = PitchCtrl.nowAbsoluteAngle;
    for (int i = 0; i < 200; ++i) unitPID_calc();
    near_value(pitch_control_debug.gravity_out, 1200 * cosf(PI / 6));
    const float held_gravity = pitch_control_debug.gravity_out;
    PitchCtrl.want_AbsoluteAngleCtrl = 0.6f;
    unitPID_calc();
    near_value(pitch_control_debug.gravity_out, held_gravity);
    PitchCtrl.want_AbsoluteAngleCtrl = 0.4f;
    unitPID_calc();
    near_value(pitch_control_debug.gravity_out, held_gravity);
    pitch_gravity_config.offset_rad = PI / 3;
    for (int i = 0; i < 200; ++i) unitPID_calc();
    near_value(pitch_control_debug.gravity_out, 0);

    /* 前馈按真实经过时间限速，异常间隔不追补；独立上限两个方向均生效。 */
    for (int period = 1; period <= 5; ++period) {
        setup();
        pitch_gravity_config.gain = 2000;
        PitchCtrl.nowTime = period;
        for (int t = 0; t < 100; t += period) unitPID_calc();
        near_value(pitch_control_debug.gravity_out,
                   fminf(2000, PITCH_GRAVITY_SLEW_PER_SECOND * period * ((100 + period - 1) / period) * 0.001f));
        const float before_pause = pitch_control_debug.gravity_out;
        PitchCtrl.nowTime = 1000;
        unitPID_calc();
        near_value(pitch_control_debug.gravity_out, before_pause);
        PitchCtrl.nowTime = 0;
        unitPID_calc();
        near_value(pitch_control_debug.gravity_out, before_pause);
    }
    for (int sign = -1; sign <= 1; sign += 2) {
        setup();
        pitch_gravity_config.gain = sign * PITCH_GRAVITY_MAX_OUT * 2;
        const int ramp_cycles = (int)(PITCH_GRAVITY_MAX_OUT / (PITCH_GRAVITY_SLEW_PER_SECOND * 0.002f)) + 2;
        for (int i = 0; i < ramp_cycles; ++i) unitPID_calc();
        near_value(pitch_control_debug.gravity_out, sign * PITCH_GRAVITY_MAX_OUT);
        assert(pitch_control_debug.gravity_limited);
        assert(fabsf((float)PitchCtrl.Volt_out) <= PITCH_GRAVITY_MAX_OUT);
        pitch_gravity_config.gain = 0;
        for (int i = 0; i < ramp_cycles; ++i) unitPID_calc();
        near_value(pitch_control_debug.gravity_out, 0);
        assert(PitchCtrl.Volt_out == 0);
    }

    /* PID 尚未饱和，但加前馈后饱和：应阻止同向积分，并限制最终总输出。 */
    for (int sign = -1; sign <= 1; sign += 2) {
        setup();
        PitchCtrl.spd_pid.Kd = 0;
        PitchCtrl.agl_pid.out = sign * (PITCH_VOLT_MAX_OUT - 1000) / PITCH_SPD_KP;
        pitch_control_debug_t debug = {0};
        near_value(pitch_speed_pid_calc(sign * 2000, &debug), sign * PITCH_VOLT_MAX_OUT);
        near_value(PitchCtrl.spd_pid.Iout, 0);
        assert(debug.integral_blocked && debug.total_saturated && !debug.pid_saturated);

        /* PID 自身超限、前馈反向抵消时，不能先截断 PID；积分应能继续。 */
        setup();
        PitchCtrl.spd_pid.Kd = 0;
        PitchCtrl.agl_pid.out = sign * (PITCH_VOLT_MAX_OUT + 1000) / PITCH_SPD_KP;
        const float expected_i = TEST_KI * PitchCtrl.agl_pid.out;
        memset(&debug, 0, sizeof(debug));
        near_value(pitch_speed_pid_calc(-sign * 2000, &debug), sign * (PITCH_VOLT_MAX_OUT - 1000) + expected_i);
        near_value(PitchCtrl.spd_pid.Iout, expected_i);
        assert(!debug.integral_blocked && !debug.total_saturated && debug.pid_saturated);
    }

    /* 总输出仍然饱和时，允许反向积分释放。 */
    setup();
    PitchCtrl.spd_pid.Kd = 0;
    PitchCtrl.spd_pid.Iout = 3000;
    PitchCtrl.spd_pid.max_out = 5000;
    PitchCtrl.agl_pid.out = -0.00002f;
    pitch_control_debug_t debug = {0};
    near_value(pitch_speed_pid_calc(3000, &debug), 5000);
    assert(PitchCtrl.spd_pid.Iout < 3000);
    assert(!debug.integral_blocked && debug.total_saturated);

    /* 已建立补偿后停机必须立即归零，重新使能从零缓升。 */
    for (int offline = 0; offline <= 1; ++offline) {
        setup();
        pitch_gravity_config.gain = 1200;
        for (int i = 0; i < 100; ++i) unitPID_calc();
        PitchCtrl.nopowerflag = !offline;
        rc_online = !offline;
        unitPID_calc();
        near_value(pitch_gravity_output, 0);
        assert(PitchCtrl.Volt_out == 0 && pitch_control_debug.motor_cmd == 0);
        assert(!pitch_control_debug.control_enabled);
        PitchCtrl.nopowerflag = 0;
        rc_online = 1;
        unitPID_calc();
        near_value(pitch_control_debug.gravity_out, PITCH_GRAVITY_SLEW_PER_SECOND * 0.002f);
        assert(PitchCtrl.Volt_out == (int16_t)(-PITCH_GRAVITY_SLEW_PER_SECOND * 0.002f));
    }

    /* 三类限幅观测分开，避免把目标限幅或积分限幅误认作总输出饱和。 */
    setup();
    PitchCtrl.want_AbsoluteAngleCtrl = -1;
    unitPID_calc();
    near_value(pitch_control_debug.requested_angle_rad, -1);
    near_value(pitch_control_debug.target_angle_rad, PITCH_MIN_ANGLE);
    assert(pitch_control_debug.target_limited);
    setup();
    PitchCtrl.spd_pid.Iout = PITCH_VOLT_MAX_IOUT;
    unitPID_calc();
    assert(pitch_control_debug.integral_limited);
    assert(!pitch_control_debug.pid_saturated && !pitch_control_debug.total_saturated);

    /* 调试器写入无效参数时撤下前馈，避免 NaN 污染电机控制量。 */
    setup();
    pitch_gravity_output = 1200;
    pitch_gravity_config.gain = NAN;
    unitPID_calc();
    assert(pitch_control_debug.gravity_config_invalid);
    assert(PitchCtrl.Volt_out == 0);
    pitch_gravity_config.gain = 1200;
    pitch_gravity_config.offset_rad = INFINITY;
    unitPID_calc();
    assert(pitch_control_debug.gravity_config_invalid);
    assert(PitchCtrl.Volt_out == 0);

    /* 积分分离：大误差时暂停积分且保留已有积分；回到阈值内恢复积分；阈值可在运行时修改。 */
    setup();
    PitchCtrl.agl_pid.Kd = 0;
    PitchCtrl.spd_pid.Iout = 1000;
    PitchCtrl.want_AbsoluteAngleCtrl = 0.3f; /* 误差 0.3 rad > 0.1 */
    for (int i = 0; i < 50; ++i) unitPID_calc();
    near_value(PitchCtrl.spd_pid.Iout, 1000);
    assert(pitch_control_debug.integral_separated);
    PitchCtrl.want_AbsoluteAngleCtrl = 0.05f; /* 误差 0.05 rad < 0.1 */
    unitPID_calc();
    assert(!pitch_control_debug.integral_separated);
    assert(PitchCtrl.spd_pid.Iout > 1000);
    pitch_i_separation_rad = 0.5f;
    PitchCtrl.want_AbsoluteAngleCtrl = 0.3f;
    const float before_wide = PitchCtrl.spd_pid.Iout;
    unitPID_calc();
    assert(!pitch_control_debug.integral_separated);
    assert(PitchCtrl.spd_pid.Iout > before_wide);
    pitch_i_separation_rad = PITCH_I_SEPARATION_RAD_DEFAULT;

    friction_tests();
    yaw_friction_tests();
    yaw_ff_tests();
    fire_gate_tests();

    puts("PASS: fire gate; Yaw feedforward; Yaw static friction; static friction fill-to-threshold; compensation-off matches baseline; integral separation; Pitch integral and gravity feedforward; combined saturation/anti-windup; ramp/disable/debug; Yaw unchanged");
}
'''
(out / 'test.c').write_text(prefix + declarations + functions + tests, encoding='utf-8')
exe = out / ('test_pitch_integral.exe' if os.name == 'nt' else 'test_pitch_integral')
subprocess.run([cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                '-I', str(out), '-I', str(root / 'Control'),
                '-I', str(root / 'Applications'), '-I', str(root / 'Task'), str(out / 'test.c'),
                str(root / 'Control/pid.c'), '-lm', '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
