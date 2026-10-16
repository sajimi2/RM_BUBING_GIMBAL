#!/usr/bin/env python3
"""提取真实的 getControlAngles/radFormat，在主机上验证视觉相对增量语义。"""

import argparse
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def extract_function(source, name):
    match = re.search(rf'^(?:void|fp32) {name}\([^\n]*\)\s*(?://[^\n]*)?\n\{{', source, re.M)
    if not match:
        raise ValueError(f'找不到函数定义：{name}')
    start = source.index('{', match.start())
    depth = 0
    for end in range(start, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if depth == 0:
            return source[match.start():end + 1]
    raise ValueError(f'函数大括号不匹配：{name}')


def extract_macro(source, name):
    lines = source.splitlines()
    for index, line in enumerate(lines):
        if re.match(rf'^#define {name}(?:\s|\()', line):
            result = [line]
            while result[-1].rstrip().endswith('\\'):
                index += 1
                result.append(lines[index])
            return '\n'.join(result)
    raise ValueError(f'找不到宏：{name}')


PREAMBLE = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
typedef float fp32;
#define PI 3.14159265358979323846f
enum RobotState_e { RobotState_e_Powerless, RobotState_e_CommonCar };
static enum RobotState_e mode, lastMode;
static const enum RobotState_e *robotMode = &mode;
static struct { fp32 nowAbsoluteAngle, want_AbsoluteAngleCtrl; uint8_t nopowerflag; }
    PitchCtrl, YawCtrl;
static struct { struct { int16_t ch[5]; uint8_t s[2]; } rc;
    struct { int16_t x, y; } mouse; } remote;
static const __typeof__(remote) *rc_p = &remote;
static struct { int16_t yaw, pitch; uint8_t mode; } vision;
static const __typeof__(vision) *nx_p = &vision;
static uint8_t data_ready;
static fp32 delta_pitch, delta_yaw;
static int auto_mode;
static int robotIsAuto(void) { return auto_mode; }
'''

TESTS = r'''
static int checks, failures;
static void near(const char *name, fp32 got, fp32 expected) {
    ++checks;
    if (!isfinite(got) || fabsf(got - expected) > 0.0001f) {
        ++failures;
        printf("FAIL %s: got %.7f, expected %.7f\n", name, got, expected);
    }
}
static void reset(void) {
    memset(&PitchCtrl, 0, sizeof(PitchCtrl));
    memset(&YawCtrl, 0, sizeof(YawCtrl));
    memset(&remote, 0, sizeof(remote));
    memset(&vision, 0, sizeof(vision));
    mode = lastMode = RobotState_e_CommonCar;
    auto_mode = 1;
    data_ready = 0;
    delta_pitch = delta_yaw = 0;
}
/* 测试输入使用弧度；这里按已约定的 CAN 缩放编码，期望值独立给定。 */
static void frame(fp32 yaw, fp32 pitch) {
    vision.yaw = (int16_t)lroundf(yaw * 32767.0f / PI);
    vision.pitch = (int16_t)lroundf(-pitch * 32767.0f / PI);
    data_ready = 1;
    getControlAngles();
}
int main(void) {
    reset();
    YawCtrl.nowAbsoluteAngle = PI / 6;  /* 30 degrees */
    YawCtrl.want_AbsoluteAngleCtrl = 70 * PI / 180;
    PitchCtrl.nowAbsoluteAngle = 0.2f;
    PitchCtrl.want_AbsoluteAngleCtrl = -0.4f;
    frame(5 * PI / 180, 0.1f);
    near("yaw 30 + 5 -> 35", YawCtrl.want_AbsoluteAngleCtrl, 35 * PI / 180);
    near("pitch feedback + positive delta", PitchCtrl.want_AbsoluteAngleCtrl, 0.3f);
    near("frame consumed", data_ready, 0);
    near("yaw delta observation in radians", delta_yaw, 5 * PI / 180);
    near("pitch delta observation in radians", delta_pitch, 0.1f);

    /* 固定反馈反复接收相同新帧，目标不应继续累加。 */
    for (int i = 0; i < 5; ++i) frame(5 * PI / 180, 0.1f);
    near("repeat frame yaw", YawCtrl.want_AbsoluteAngleCtrl, 35 * PI / 180);
    near("repeat frame pitch", PitchCtrl.want_AbsoluteAngleCtrl, 0.3f);

    /* 反馈变化但没有新帧时，不能再把旧增量加到新反馈上。 */
    YawCtrl.nowAbsoluteAngle = -0.7f;
    PitchCtrl.nowAbsoluteAngle = -0.2f;
    vision.yaw = vision.pitch = 0;
    getControlAngles();
    near("hold yaw without new frame", YawCtrl.want_AbsoluteAngleCtrl, 35 * PI / 180);
    near("hold pitch without new frame", PitchCtrl.want_AbsoluteAngleCtrl, 0.3f);
    frame(-0.1f, -0.15f);
    near("new frame uses latest yaw feedback", YawCtrl.want_AbsoluteAngleCtrl, -0.8f);
    near("new frame uses latest pitch feedback", PitchCtrl.want_AbsoluteAngleCtrl, -0.35f);
    frame(0, 0);
    near("zero yaw delta anchors current", YawCtrl.want_AbsoluteAngleCtrl, -0.7f);
    near("zero pitch delta anchors current", PitchCtrl.want_AbsoluteAngleCtrl, -0.2f);

    reset();
    YawCtrl.nowAbsoluteAngle = 179 * PI / 180;
    frame(5 * PI / 180, 0);
    near("yaw wraps across +pi", YawCtrl.want_AbsoluteAngleCtrl, -176 * PI / 180);
    near("wrapped positive yaw error", radFormat(YawCtrl.want_AbsoluteAngleCtrl - YawCtrl.nowAbsoluteAngle), 5 * PI / 180);
    YawCtrl.nowAbsoluteAngle = -179 * PI / 180;
    frame(-5 * PI / 180, 0);
    near("yaw wraps across -pi", YawCtrl.want_AbsoluteAngleCtrl, 176 * PI / 180);
    near("wrapped negative yaw error", radFormat(YawCtrl.want_AbsoluteAngleCtrl - YawCtrl.nowAbsoluteAngle), -5 * PI / 180);

    reset();
    auto_mode = 0;
    remote.rc.s[1] = 2;
    YawCtrl.nowAbsoluteAngle = 0.3f;
    frame(0.1f, 0);
    near("switch selects visual control", YawCtrl.want_AbsoluteAngleCtrl, 0.4f);

    reset();
    auto_mode = 0;
    remote.rc.ch[0] = remote.rc.ch[1] = 100;
    YawCtrl.want_AbsoluteAngleCtrl = 0.2f;
    PitchCtrl.want_AbsoluteAngleCtrl = 0.1f;
    frame(0.5f, 0.5f);
    near("manual yaw unchanged", YawCtrl.want_AbsoluteAngleCtrl, 0.198f);
    near("manual pitch unchanged", PitchCtrl.want_AbsoluteAngleCtrl, 0.0995f);
    near("manual does not consume vision frame", data_ready, 1);

    reset();
    mode = RobotState_e_Powerless;
    YawCtrl.want_AbsoluteAngleCtrl = 0.3f;
    PitchCtrl.want_AbsoluteAngleCtrl = -0.2f;
    frame(0.5f, 0.5f);
    near("powerless yaw target unchanged", YawCtrl.want_AbsoluteAngleCtrl, 0.3f);
    near("powerless pitch target unchanged", PitchCtrl.want_AbsoluteAngleCtrl, -0.2f);
    near("powerless yaw flag", YawCtrl.nopowerflag, 1);
    near("powerless pitch flag", PitchCtrl.nopowerflag, 1);
    near("powerless does not consume frame", data_ready, 1);

    reset();
    lastMode = RobotState_e_Powerless;
    YawCtrl.nowAbsoluteAngle = 0.7f;
    PitchCtrl.nowAbsoluteAngle = 0.2f;
    getControlAngles();
    near("enable anchors yaw", YawCtrl.want_AbsoluteAngleCtrl, 0.7f);
    near("enable anchors pitch", PitchCtrl.want_AbsoluteAngleCtrl, 0.2f);
    printf("%s: %d checks, %d failures\n", failures ? "FAIL" : "PASS", checks, failures);
    return failures != 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'Task/gimbal_task.c')
    parser.add_argument('--cc', help='主机 GCC 路径（不能使用 arm-none-eabi-gcc）')
    args = parser.parse_args()
    cc = args.cc or os.environ.get('CC') or shutil.which('gcc')
    if not cc and Path(r'C:\c-dev\w64devkit\bin\gcc.exe').is_file():
        cc = r'C:\c-dev\w64devkit\bin\gcc.exe'
    if not cc:
        parser.error('未找到主机 GCC；请通过 --cc 指定。')
    source = args.source.read_text(encoding='utf-8-sig')
    macros = ('RC_DEADLINE', 'PITCH_RC_SEN', 'YAW_RC_SEN',
              'PITCH_MOUSE_SEN', 'YAW_MOUSE_SEN', 'rc_deadband_limit')
    harness = '\n'.join([PREAMBLE, *(extract_macro(source, name) for name in macros),
                         extract_function(source, 'radFormat'),
                         extract_function(source, 'getControlAngles'), TESTS])
    print(f'源码：{args.source.resolve()}', flush=True)
    print(f'SHA-256：{hashlib.sha256(args.source.read_bytes()).hexdigest()}', flush=True)
    with tempfile.TemporaryDirectory(prefix='vision_relative_') as temp:
        c_file = Path(temp) / 'check.c'
        executable = Path(temp) / ('check.exe' if os.name == 'nt' else 'check')
        c_file.write_text(harness, encoding='utf-8')
        subprocess.run([cc, '-std=gnu99', '-Wall', '-Wextra', '-Werror', '-O0',
                        str(c_file), '-o', str(executable), '-lm'], check=True)
        return subprocess.run([str(executable)], check=False).returncode


if __name__ == '__main__':
    raise SystemExit(main())
