"""直接编译当前 VOFA 协议、服务和真实控制入口，不连接硬件。"""
from pathlib import Path
import os
import re
import shutil
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/host-tuning'


def function(source, name):
    match = re.search(r'^(?:static )?(?:uint8_t|void) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    source = (ROOT / 'Task/gimbal_task.c').read_text(encoding='utf-8-sig')
    nx = (ROOT / 'Task/NX_communicate.c').read_text(encoding='utf-8-sig')
    (OUT / 'main.h').write_text('#include <stddef.h>\n', encoding='utf-8')
    (OUT / 'cmsis_compiler.h').write_text('#define __PACKED_STRUCT struct __attribute__((packed))\n')
    (OUT / 'bsp_RC.h').write_text('/* no receiver hardware in host tests */\n')
    declarations = source[source.index('#define GIMBAL_TASK_CTRL_TIME'):source.index('// 模式相关')]
    prefix = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "Robot_Information.h"
#include "NX_communicate.h"
#include "tuning_service.h"
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
#define PI 3.14159265358979323846f
typedef struct { float unused; } first_order_filter_type_t;
static uint32_t irq_key;
static uint32_t __get_PRIMASK(void) { return irq_key; }
static void __disable_irq(void) { irq_key = 1; }
static void __set_PRIMASK(uint32_t key) { irq_key = key; }
static uint32_t xTaskGetTickCount(void) { return 1234; }
static uint8_t auto_mode;
static uint8_t robotIsAuto(void) { return auto_mode; }
volatile uint8_t rc_online = 1;
static enum RobotState_e mode = RobotState_e_CommonCar;
static const enum RobotState_e *robotMode = &mode;
static enum RobotState_e lastMode = RobotState_e_CommonCar;
static RC_ctrl_t rc;
static const RC_ctrl_t *rc_p = &rc;
float delta_pitch, delta_yaw;
CAN_Send_to_MCU NX_data;
volatile uint8_t data_ready;
static volatile uint8_t input_source = NX_SOURCE_CAN;
uint8_t fire_t;
'''
    funcs = '\n'.join(function(nx, name) for name in (
        'NX_GetSource', 'NX_SelectSource', 'NX_ReceiveFrame', 'NX_TakeControlFrame'))
    funcs += '\n' + '\n'.join(function(source, name) for name in (
        'recordFireFrame', 'getControlAngles', 'gimbal_tuning_is_enabled', 'gimbal_tuning_get_state',
        'gimbal_tuning_get_limits', 'gimbal_tuning_set_gravity', 'gimbal_tuning_set_parameters',
        'gimbal_tuning_set_source', 'gimbal_tuning_submit_vision'))
    tests = r'''
static tuning_parser_t parser;
static float channels[TUNING_CHANNEL_COUNT];
static uint8_t frame[TUNING_FRAME_SIZE];
static void snapshot(void)
{
    assert(tuning_service_frame(frame) == TUNING_FRAME_SIZE);
    assert(memcmp(frame + TUNING_FRAME_SIZE - 4, "\0\0\x80\x7f", 4) == 0);
    memcpy(channels, frame, sizeof(channels)); /* host and target both little endian */
}
static unsigned send_bytes(const char *text, size_t length)
{
    tuning_message_t message;
    unsigned count = 0;
    for (size_t i = 0; i < length; ++i) {
        if (tuning_parser_feed(&parser, (uint8_t)text[i], &message)) {
            tuning_service_request(&message);
            ++count;
        }
    }
    snapshot();
    return count;
}
static void command(const char *text, unsigned expected)
{
    assert(send_bytes(text, strlen(text)) == 1);
    if (channels[34] != expected)
        fprintf(stderr, "%s expected status %u, got %g\n", text, expected, channels[34]);
    assert(channels[34] == expected);
}
static void near_value(float actual, float expected)
{
    if (fabsf(actual - expected) >= 0.0002f)
        fprintf(stderr, "actual=%g expected=%g\n", actual, expected);
    assert(fabsf(actual - expected) < 0.0002f);
}
int main(int argc, char **argv)
{
    (void)argv;
    memset(&PitchCtrl, 0, sizeof(PitchCtrl));
    memset(&YawCtrl, 0, sizeof(YawCtrl));
    snapshot();
    assert(channels[36] == 20 && channels[40] == 1);
    near_value(channels[38], PITCH_MIN_ANGLE * 180 / PI);
    near_value(channels[32], 1.234f);
    command("parameters:2500,66,6000\n", TUNING_OK);
    assert(channels[15] == 2500 && channels[17] == 6000);
    near_value(channels[16], 66);
    command("parameters:1000,30,-1\n", TUNING_OUT_OF_RANGE);
    assert(channels[15] == 2500 && channels[17] == 6000);
    near_value(channels[16], 66);
    PitchCtrl.spd_pid.Iout = 100;
    command("ki:4000\n", TUNING_OK);
    assert(PitchCtrl.spd_pid.Iout == 100);
    command("ki:0\n", TUNING_OK);
    assert(PitchCtrl.spd_pid.Iout == 0);
    command("ki:8001\n", TUNING_OUT_OF_RANGE);
    command("offset:181\n", TUNING_OUT_OF_RANGE);
    command("gain:5001\n", TUNING_OUT_OF_RANGE);
    command("gravity:0\n", TUNING_OK);
    assert(channels[15] == 0);
    command("gravity:0\n", TUNING_OK);
    command("gravity:1\n", TUNING_OK);
    assert(channels[15] == 2500);
    command("gravity:1\n", TUNING_BAD_PARAMETER);
    command("gravity:0\n", TUNING_OK);
    command("gain:1200\n", TUNING_OK);
    command("gravity:1\n", TUNING_BAD_PARAMETER);
    command("parameters:2500,66,6000\n", TUNING_OK);
    command("source:1\n", TUNING_NOT_ENABLED);
    assert(channels[18] == 0);
    rc.rc.s[1] = 2;
    rc_online = 0;
    command("source:1\n", TUNING_NOT_ENABLED);
    rc_online = 1;
    mode = RobotState_e_Powerless;
    command("source:1\n", TUNING_NOT_ENABLED);
    mode = RobotState_e_CommonCar;
    PitchCtrl.nowAbsoluteAngle = 0.1f;
    pitch_control_debug.actual_angle_rad = 0.1f;
    YawCtrl.want_AbsoluteAngleCtrl = 0.3f;
    command("source:1\n", TUNING_OK);
    getControlAngles();
    near_value(PitchCtrl.want_AbsoluteAngleCtrl, 0.1f);
    near_value(YawCtrl.want_AbsoluteAngleCtrl, 0.3f);
    command("target:5\n", TUNING_OK);
    getControlAngles();
    near_value(PitchCtrl.want_AbsoluteAngleCtrl, 5 * PI / 180);
    near_value(YawCtrl.want_AbsoluteAngleCtrl, 0.3f);
    assert(NX_data.fire == 0);
    command("source:1\n", TUNING_OK);
    getControlAngles();
    near_value(PitchCtrl.want_AbsoluteAngleCtrl, 5 * PI / 180);
    command("target:-90\n", TUNING_OUT_OF_RANGE);
    command("target:90\n", TUNING_OUT_OF_RANGE);
    char boundary[60];
    snprintf(boundary, sizeof(boundary), "target:%.8f\n", (double)(PITCH_MIN_ANGLE * 180 / PI));
    command(boundary, TUNING_OK);
    getControlAngles();
    assert(PitchCtrl.want_AbsoluteAngleCtrl >= PITCH_MIN_ANGLE);
    rc.rc.s[1] = 1;
    getControlAngles();
    assert(NX_GetSource() == NX_SOURCE_CAN);
    command("target:0\n", TUNING_NOT_ENABLED);
    rc.rc.s[1] = 2;
    command("target:0\n", TUNING_WRONG_SOURCE);
    command("source:0\n", TUNING_OK);
    /* 保留新门控的集成检查：CAN 的 fire/距离与本帧角度一起记录。 */
    const uint8_t can_payload[8] = {1, 0, 0, 0, 0, 1, 0xe8, 0x03};
    assert(NX_ReceiveFrame(can_payload, NX_SOURCE_CAN));
    getControlAngles();
    assert(fire_frame_valid && fire_frame_source == NX_SOURCE_CAN);
    assert(fire_frame_fire == 1 && fire_frame_rev == 1000);
    assert(fire_frame_time == PitchCtrl.nowTime);
    /* VOFA 串口模拟仍只瞄准，不能把上一帧 CAN 的开火标志带入。 */
    command("source:1\n", TUNING_OK);
    getControlAngles();
    assert(fire_frame_source == NX_SOURCE_UART);
    assert(fire_frame_fire == 0 && fire_frame_rev == 0);
    command("source:0\n", TUNING_OK);
    command("source:0.5\n", TUNING_BAD_PARAMETER);
    command("stream:0\n", TUNING_OK);
    assert(tuning_service_period_ms() == 0);
    command("read:0\n", TUNING_OK);
    command("stream:19\n", TUNING_OUT_OF_RANGE);
    command("stream:20.5\n", TUNING_BAD_PARAMETER);
    command("stream:2001\n", TUNING_OUT_OF_RANGE);
    command("stream:20\n", TUNING_OK);
    command("mark:42\n", TUNING_OK);
    assert(channels[37] == 42);
    command("mark:-1\n", TUNING_OUT_OF_RANGE);
    command("unknown:1\n", TUNING_BAD_COMMAND);
    command("gain:12junk\n", TUNING_BAD_LENGTH);
    command("gain:NaN\n", TUNING_BAD_LENGTH);
    command("gain:inf\n", TUNING_BAD_LENGTH);
    command("gain:1e50\n", TUNING_OUT_OF_RANGE);
    command("gain:0x1p2\n", TUNING_BAD_LENGTH);
    command("gain:1,2\n", TUNING_BAD_LENGTH);
    command("gain:1,2,3,4\n", TUNING_BAD_LENGTH);
    command("gain:1,\n", TUNING_BAD_LENGTH);
    command("gain:\n", TUNING_BAD_LENGTH);
    command("gain: +1200.0 \r\n", TUNING_OK);
    assert(send_bytes("\n\r\n", 3) == 0);
    assert(channels[15] == 1200);
    assert(send_bytes("gain:10", 7) == 0);
    assert(send_bytes("00\n", 3) == 1);
    assert(channels[15] == 1000);
    tuning_parser_discard(&parser);
    command("gain:5000\n", TUNING_BAD_LENGTH);
    assert(channels[15] == 1000);
    const char invalid[] = {'g','a','i','n',':','1',0,'0','\n'};
    assert(send_bytes(invalid, sizeof(invalid)) == 1 && channels[34] == TUNING_BAD_LENGTH);
    char overflow[150];
    memset(overflow, 'x', sizeof(overflow));
    memcpy(overflow + 130, "gain:5000\n", 10);
    assert(send_bytes(overflow, 140) == 1 && channels[34] == TUNING_BAD_LENGTH);
    assert(channels[15] == 1000);
    command("read:0\n", TUNING_OK);
    assert(irq_key == 0);
    pitch_control_debug.friction_out = 123;
    snapshot();
    assert(channels[11] == 123);
    pitch_control_debug.d_out = INFINITY;
    snapshot();
    assert(channels[7] == 0 && channels[40] == 0);
    pitch_control_debug.d_out = 17;
    command("read:0\n", TUNING_OK);
    assert(channels[40] == 1);
    if (argc > 1) {
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        assert(fwrite(frame, 1, sizeof(frame), stdout) == sizeof(frame));
    } else {
        puts("PASS: VOFA codec/service + actual control gates/atomic parameters/Ki/gravity/source/targets/nonfinite handling");
    }
    return 0;
}
'''
    (OUT / 'test.c').write_text(prefix + declarations + funcs + tests, encoding='utf-8')
    executable = OUT / ('test_tuning.exe' if os.name == 'nt' else 'test_tuning')
    cc = os.environ.get('HOST_CC') or shutil.which('gcc')
    assert cc, 'Host GCC required'
    subprocess.run([cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-variable',
                    '-I', str(OUT), '-I', str(ROOT / 'Control'), '-I', str(ROOT / 'Task'),
                    '-I', str(ROOT / 'Applications'), '-I', str(ROOT / 'Debug'),
                    str(OUT / 'test.c'), str(ROOT / 'Debug/tuning_protocol.c'),
                    str(ROOT / 'Debug/tuning_service.c'), '-lm', '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
    frame = subprocess.check_output([str(executable), 'frame'])
    assert len(frame) == 168 and frame[-4:] == b'\x00\x00\x80\x7f'
    values = struct.unpack('<41f', frame[:-4])
    assert values[7] == 17 and values[15] == 1000 and values[37] == 42
    print('PASS: independently decoded 41-channel little-endian JustFloat frame')


if __name__ == '__main__':
    main()
