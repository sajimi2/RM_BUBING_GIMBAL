"""宿主验证真实 Pitch 控制函数及 PID 实现；不模拟或宣称实机稳定性。"""
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
    match = re.search(r'^(?:static )?(?:fp32|void) ' + name +
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
#define PI 3.14159265358979323846f
typedef struct { float unused; } first_order_filter_type_t;
volatile uint8_t rc_online = 1;
'''
declarations = source[source.index('#define GIMBAL_TASK_CTRL_TIME'):
                      source.index('// 模式相关')]
functions = '\n'.join(function(name) for name in (
    'radFormat', 'gimbal_PID_calc', 'pitch_speed_pid_calc', 'unitPID_calc'))
tests = r'''
static void near_value(float a, float b)
{
    assert(fabsf(a - b) < 0.002f);
}

static void setup(void)
{
    memset(&PitchCtrl, 0, sizeof(PitchCtrl));
    memset(&YawCtrl, 0, sizeof(YawCtrl));
    rc_online = 1;
    PID_init(&PitchCtrl.spd_pid, PID_POSITION, pitchSpdPIDco,
             PITCH_VOLT_MAX_OUT, PITCH_VOLT_MAX_IOUT);
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

static float integrate_for_ms(uint32_t period_ms, int duration_ms)
{
    setup();
    PitchCtrl.agl_pid.Kd = 0;
    PitchCtrl.want_AbsoluteAngleCtrl = 0.01f;
    PitchCtrl.nowTime = period_ms;
    for (int t = 0; t < duration_ms; t += (int)period_ms)
        unitPID_calc();
    near_value(PitchCtrl.spd_pid.Ki, PITCH_SPD_KI);
    return PitchCtrl.spd_pid.Iout;
}

int main(void)
{
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
    puts("PASS: Pitch integral/time scaling/limits/anti-windup/disable/re-enable/tick wrap; Yaw unchanged");
}
'''
(out / 'test.c').write_text(prefix + declarations + functions + tests, encoding='utf-8')
exe = out / ('test_pitch_integral.exe' if os.name == 'nt' else 'test_pitch_integral')
subprocess.run([cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                '-I', str(out), '-I', str(root / 'Control'),
                '-I', str(root / 'Applications'), str(out / 'test.c'),
                str(root / 'Control/pid.c'), '-lm', '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
