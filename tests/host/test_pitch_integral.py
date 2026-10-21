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
#define PI 3.14159265358979323846f
typedef struct { float unused; } first_order_filter_type_t;
volatile uint8_t rc_online = 1;
'''
declarations = source[source.index('#define GIMBAL_TASK_CTRL_TIME'):
                      source.index('// 模式相关')]
functions = '\n'.join(function(name) for name in (
    'radFormat', 'gimbal_PID_calc', 'calcPitchGravity',
    'restartPitchStillTimer', 'isPitchStill', 'checkFrictionThreshold', 'calcPitchFriction',
    'limitPitchOutput', 'pitch_speed_pid_calc', 'unitPID_calc'))
tests = r'''
static void near_value_at(float a, float b, int line)
{
    if (fabsf(a - b) >= 0.002f) fprintf(stderr, "near_value failed at generated line %d: %f != %f\n", line, a, b);
    assert(fabsf(a - b) < 0.002f);
}
#define near_value(a, b) near_value_at((a), (b), __LINE__)

#define TEST_KI 8000.0f /* 仅用于验证积分路径，不是工程默认值 */

static int16_t test_pitch_rpm; /* 模拟 Pitch 6020 的整数 rpm 反馈 */

static void setup_default(void)
{
    memset(&PitchCtrl, 0, sizeof(PitchCtrl));
    memset(&YawCtrl, 0, sizeof(YawCtrl));
    memset((void *)&pitch_control_debug, 0, sizeof(pitch_control_debug));
    pitch_gravity_config.gain = 0;
    pitch_gravity_config.offset_rad = 0;
    pitch_gravity_output = 0;
    /* 摩擦补偿保持工程默认（门槛0=关闭），基线用例在补偿代码运行时验证输出不变。 */
    pitch_friction_up_out = PITCH_FRICTION_UP_OUT_DEFAULT;
    pitch_friction_down_out = PITCH_FRICTION_DOWN_OUT_DEFAULT;
    pitch_friction_error_rad = PITCH_FRICTION_ERROR_RAD_DEFAULT;
    pitch_still_valid = 0;
    test_pitch_rpm = 0;
    PitchCtrl.SpeedRpmPoint = &test_pitch_rpm;
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
        pitch_control_debug_t check = {0};
        PitchCtrl.agl_pid.error[0] = 0.01f;
        PitchCtrl.agl_pid.out = 0.0001f;
        near_value(calcPitchFriction(600, &check), 150);
        PitchCtrl.agl_pid.out = -0.0001f;
        near_value(calcPitchFriction(600, &check), 0);
        PitchCtrl.agl_pid.error[0] = -0.01f;
        PitchCtrl.agl_pid.out = -0.0001f;
        near_value(calcPitchFriction(-600, &check), -400);
        near_value(calcPitchFriction(200, &check), 0); /* 低头时 PID 在往上推（刹车）：不补 */
        PitchCtrl.agl_pid.out = 0.0001f;
        near_value(calcPitchFriction(-600, &check), 0);
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
    const float bad_values[] = {NAN, INFINITY, -750, PITCH_FRICTION_MAX_OUT + 1};
    for (int i = 0; i < 4; ++i) {
        setup_friction(0.01f);
        pitch_friction_up_out = bad_values[i];
        run_ms(200);
        assert(!pitch_control_debug.friction_active);
        near_value(pitch_control_debug.total_out, 600);
    }
    setup_friction(0.01f);
    pitch_friction_up_out = PITCH_FRICTION_MAX_OUT;
    run_ms(200);
    near_value(pitch_control_debug.total_out, PITCH_FRICTION_MAX_OUT);
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

    puts("PASS: static friction fill-to-threshold; compensation-off matches baseline; integral separation; Pitch integral and gravity feedforward; combined saturation/anti-windup; ramp/disable/debug; Yaw unchanged");
}
'''
(out / 'test.c').write_text(prefix + declarations + functions + tests, encoding='utf-8')
exe = out / ('test_pitch_integral.exe' if os.name == 'nt' else 'test_pitch_integral')
subprocess.run([cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                '-I', str(out), '-I', str(root / 'Control'),
                '-I', str(root / 'Applications'), '-I', str(root / 'Task'), str(out / 'test.c'),
                str(root / 'Control/pid.c'), '-lm', '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
