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
    match = re.search(r'^(?:static )?(?:uint8_t|void|fp32) ' + name +
                      r'\([^;]*?\)\s*(?://[^\n]*\n\s*)?\{', source, re.M)
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
static uint32_t now_ms = 1234;
static uint32_t HAL_GetTick(void) { return now_ms; }
static uint32_t xTaskGetTickCount(void) { return now_ms; }
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
static volatile uint8_t can_frame_seen;
static volatile uint32_t last_can_frame_ms;
uint8_t fire_t;
static uint8_t mock_friction_enabled;
static motor_measure_t mock_friction_motors[2];
static float mock_gyro_mean[3], mock_gyro_std[3];
static unsigned gyro_stats_reads;
const RC_ctrl_t *get_remote_control_point(void) { return &rc; }
uint8_t get_fric(void) { return mock_friction_enabled; }
const motor_measure_t *get_shoot_motor_measure_point(uint8_t motor)
{
    return &mock_friction_motors[motor];
}
void INS_GetGyroStats(float mean[3], float std[3])
{
    ++gyro_stats_reads;
    memcpy(mean, mock_gyro_mean, sizeof(mock_gyro_mean));
    memcpy(std, mock_gyro_std, sizeof(mock_gyro_std));
}
'''
    funcs = '\n'.join(function(nx, name) for name in (
        'NX_GetSource', 'NX_VisionOnline', 'NX_SelectSource', 'NX_ReceiveFrame', 'NX_TakeControlFrame'))
    for name in (
        'radFormat', 'ECD2Rad', 'gimbal_tuning_is_enabled', 'updateYawWave', 'recordFireFrame', 'recordYawFrame',
        'updateYawVisionMotion', 'getControlAngles', 'getTuningDiagnostics', 'gimbal_tuning_get_state',
        'gimbal_tuning_get_limits', 'gimbal_tuning_set_gravity', 'gimbal_tuning_set_parameters',
        'gimbal_tuning_stop_yaw_wave', 'gimbal_tuning_set_source', 'gimbal_tuning_submit_vision',
        'gimbal_tuning_start_yaw_wave', 'gimbal_tuning_keep_yaw_wave'):
        body = function(source, name)
        if name == 'getTuningDiagnostics':
            body = '#if TUNING_DIAGNOSTICS_ENABLED && TUNING_VOFA_TX_ENABLED\n' + body + '\n#endif\n'
        funcs += '\n' + body
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
    if (last_status != expected)
        fprintf(stderr, "%s expected status %u, got %g\n", text, expected, (double)last_status);
    assert(last_status == expected);
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
#if TUNING_DIAGNOSTICS_ENABLED && TUNING_VOFA_TX_ENABLED
    assert(TUNING_CHANNEL_COUNT == 23 && TUNING_FRAME_SIZE == 96);
    assert(channels[12] == 0); /* 云台初始化前空编码器指针也可读取遥测。 */
#else
    assert(TUNING_CHANNEL_COUNT == 12 && TUNING_FRAME_SIZE == 52);
    assert(gyro_stats_reads == 0);
#endif
    assert(tuning_service_period_ms() == 20 && channels[5] == 0);
    command("parameters:2500,66,6000\n", TUNING_OK);
    assert(pitch_gravity_config.gain == 2500 && PitchCtrl.spd_pid.Ki == 6000);
    near_value((pitch_gravity_config.offset_rad * 180 / PI), 66);
    command("parameters:1000,30,-1\n", TUNING_OUT_OF_RANGE);
    assert(pitch_gravity_config.gain == 2500 && PitchCtrl.spd_pid.Ki == 6000);
    near_value((pitch_gravity_config.offset_rad * 180 / PI), 66);
    PitchCtrl.spd_pid.Iout = 100;
    command("ki:4000\n", TUNING_OK);
    assert(PitchCtrl.spd_pid.Iout == 100);
    command("ki:0\n", TUNING_OK);
    assert(PitchCtrl.spd_pid.Iout == 0);
    command("ki:8001\n", TUNING_OUT_OF_RANGE);
    command("offset:181\n", TUNING_OUT_OF_RANGE);
    command("gain:5001\n", TUNING_OUT_OF_RANGE);
    command("gravity:0\n", TUNING_OK);
    assert(pitch_gravity_config.gain == 0);
    command("gravity:0\n", TUNING_OK);
    command("gravity:1\n", TUNING_OK);
    assert(pitch_gravity_config.gain == 2500);
    command("gravity:1\n", TUNING_BAD_PARAMETER);
    command("gravity:0\n", TUNING_OK);
    command("gain:1200\n", TUNING_OK);
    command("gravity:1\n", TUNING_BAD_PARAMETER);
    command("parameters:2500,66,6000\n", TUNING_OK);
    command("source:1\n", TUNING_NOT_ENABLED);
    command("yaw_wave:3,30,10\n", TUNING_NOT_ENABLED);
    assert(NX_GetSource() == 0);
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
    /* 连续Yaw绝对目标不累加误差，也不能改变已保持的Pitch目标。 */
    pitch_control_debug.target_angle_rad = PitchCtrl.want_AbsoluteAngleCtrl;
    for (unsigned sample = 0; sample < 48; ++sample) {
        float yaw_deg = 20 + 3 * sinf(2 * PI * 0.5f * sample / 24);
        char yaw_command[50];
        snprintf(yaw_command, sizeof(yaw_command), "yaw:%.5f\n", (double)yaw_deg);
        command(yaw_command, TUNING_OK);
        getControlAngles();
        near_value(YawCtrl.want_AbsoluteAngleCtrl, yaw_deg * PI / 180);
        near_value(PitchCtrl.want_AbsoluteAngleCtrl, 5 * PI / 180);
        assert(NX_data.fire == 0 && NX_data.rev == 0);
    }
    command("yaw:181\n", TUNING_OUT_OF_RANGE);
    /* 高频目标由2 ms控制路径生成，30 Hz一秒应有30轮，且不能改Pitch。 */
    YawCtrl.nowAbsoluteAngle = 0.2f;
    command("yaw_wave:3,30.1,1\n", TUNING_OUT_OF_RANGE);
    command("yaw_wave:0,30,1\n", TUNING_OUT_OF_RANGE);
    command("yaw_wave:3,30,0\n", TUNING_OUT_OF_RANGE);
    command("yaw_wave:3,30,1\n", TUNING_OK);
    uint32_t wave_start = now_ms;
    for (unsigned tick = 0; tick < 1000; tick += 2) {
        now_ms = wave_start + tick;
        if (tick % 100 == 0)
            command("yaw_ping:0\n", TUNING_OK);
        getControlAngles();
        near_value(YawCtrl.want_AbsoluteAngleCtrl, 0.2f + 3 * PI / 180 * sinf(2 * PI * 30 * tick / 1000));
        /* 波形测试时，前馈用的目标角速度、角加速度按公式精确给出。 */
        assert(yaw_target_source == YAW_TARGET_SOURCE_WAVE);
        near_value(yaw_target_speed / 100, 3 * PI / 180 * 2 * PI * 30 * cosf(2 * PI * 30 * tick / 1000) / 100);
        near_value(yaw_target_accel / 10000, -3 * PI / 180 * 2 * PI * 30 * 2 * PI * 30 * sinf(2 * PI * 30 * tick / 1000) / 10000);
        near_value(PitchCtrl.want_AbsoluteAngleCtrl, 5 * PI / 180);
        assert(NX_data.fire == 0 && NX_data.rev == 0);
    }
    float last_wave_target = YawCtrl.want_AbsoluteAngleCtrl;
    now_ms = wave_start + 1000;
    getControlAngles();
    assert(!yaw_wave.active);
    near_value(YawCtrl.want_AbsoluteAngleCtrl, last_wave_target);
    command("yaw_ping:0\n", TUNING_NOT_ENABLED);
    command("yaw_wave:3,30,10\n", TUNING_OK);
    now_ms += 500;
    getControlAngles();
    assert(!yaw_wave.active); /* 断连或窗口关闭，无保活就停止。 */
    command("yaw_wave:3,30,10\n", TUNING_OK);
    command("yaw:5\n", TUNING_OK);
    getControlAngles();
    assert(!yaw_wave.active);
    near_value(YawCtrl.want_AbsoluteAngleCtrl, 5 * PI / 180);
    command("yaw_wave:3,30,10\n", TUNING_OK);
    command("yaw_stop:0\n", TUNING_OK);
    assert(!yaw_wave.active);
    command("yaw_stop:1\n", TUNING_BAD_PARAMETER);
    command("yaw_wave:3,30,10\n", TUNING_OK);
    rc_online = 0;
    getControlAngles();
    assert(!yaw_wave.active && NX_GetSource() == NX_SOURCE_CAN);
    rc_online = 1;
    command("yaw_wave:3,30,10\n", TUNING_WRONG_SOURCE);
    command("source:1\n", TUNING_OK);
    getControlAngles();
    now_ms = 1234; /* 后面的CAN在线超时用例继续使用原来的时基。 */
    command("target:5\n", TUNING_OK);
    getControlAngles();
    command("yaw:-181\n", TUNING_OUT_OF_RANGE);
    command("yaw:179\n", TUNING_OK);
    getControlAngles();
    near_value(YawCtrl.want_AbsoluteAngleCtrl, 179 * PI / 180);
    command("yaw:-179\n", TUNING_OK);
    getControlAngles();
    near_value(YawCtrl.want_AbsoluteAngleCtrl, -179 * PI / 180);
    YawCtrl.want_AbsoluteAngleCtrl = 0.3f;
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
    assert(yaw_frame_valid); /* 自瞄时收过帧 */
    rc.rc.s[1] = 1;
    getControlAngles();
    assert(NX_GetSource() == NX_SOURCE_CAN);
    assert(!yaw_frame_valid); /* 退出自瞄清掉帧记录，重新进入后不拿旧帧估速度 */
    command("target:0\n", TUNING_NOT_ENABLED);
    command("yaw:0\n", TUNING_NOT_ENABLED);
    rc.rc.s[1] = 2;
    command("target:0\n", TUNING_WRONG_SOURCE);
    command("yaw:0\n", TUNING_WRONG_SOURCE);
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
    command("stream:4\n", TUNING_OUT_OF_RANGE);
    command("stream:5\n", TUNING_OK);
    command("stream:19\n", TUNING_OK);
    command("stream:20\n", TUNING_OK);
    command("stream:24\n", TUNING_OK);
    command("stream:20.5\n", TUNING_BAD_PARAMETER);
    command("stream:2001\n", TUNING_OUT_OF_RANGE);
    command("stream:25\n", TUNING_OK);
    command("mark:42\n", TUNING_OK);
    assert(record_mark == 42);
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
    assert(pitch_gravity_config.gain == 1200);
    assert(send_bytes("gain:10", 7) == 0);
    assert(send_bytes("00\n", 3) == 1);
    assert(pitch_gravity_config.gain == 1000);
    tuning_parser_discard(&parser);
    command("gain:5000\n", TUNING_BAD_LENGTH);
    assert(pitch_gravity_config.gain == 1000);
    const char invalid[] = {'g','a','i','n',':','1',0,'0','\n'};
    assert(send_bytes(invalid, sizeof(invalid)) == 1 && last_status == TUNING_BAD_LENGTH);
    char overflow[150];
    memset(overflow, 'x', sizeof(overflow));
    memcpy(overflow + 130, "gain:5000\n", 10);
    assert(send_bytes(overflow, 140) == 1 && last_status == TUNING_BAD_LENGTH);
    assert(pitch_gravity_config.gain == 1000);
    command("read:0\n", TUNING_OK);
    assert(irq_key == 0);
    /* 前十二通道顺序、单位和最终输出保持不变；快照不改写控制。 */
    pitch_control_debug.actual_angle_rad = -0.25f;
    pitch_control_debug.target_angle_rad = 0.5f;
    YawCtrl.nowAbsoluteAngle = -179 * PI / 180;
    YawCtrl.want_AbsoluteAngleCtrl = 179 * PI / 180;
    YawCtrl.agl_pid.out = 0.0005f;
    yaw_speed_setpoint = 0.0005f; /* 速度环目标 = 位置环输出 + 速度前馈；前馈关闭时两者相同 */
    YawCtrl.radspeed = -0.00025f;
    YawCtrl.spd_pid.Pout = 31000;
    YawCtrl.Volt_out = 20000;
    fire_gate_debug.yaw_error_rad = 0.012f;
    fire_gate_debug.yaw_threshold_rad = 0.047f / 3.0f;
    fire_gate_debug.pitch_error_rad = 0.008f;
    fire_gate_debug.pitch_threshold_rad = 0.043f / 3.0f;
    uint16_t pitch_ecd = 6144;
    PitchCtrl.ECDPoint = &pitch_ecd;
    rc.rc.s[1] = 2;
    mock_friction_enabled = 1;
    mock_friction_motors[0].speed_rpm = 6000;
    mock_friction_motors[1].speed_rpm = -6100;
    mock_gyro_mean[0] = 2.0f;
    mock_gyro_mean[1] = -3.0f;
    mock_gyro_mean[2] = 0.5f;
    mock_gyro_std[0] = 0.25f;
    mock_gyro_std[1] = 0.5f;
    mock_gyro_std[2] = 1.0f;
    fire_gate_debug_t fire_before = fire_gate_debug;
    struct GimbalMotorCtrl_s yaw_before = YawCtrl;
    struct GimbalMotorCtrl_s pitch_before = PitchCtrl;
    snapshot();
    near_value(channels[0], -0.25f * 180 / PI);
    near_value(channels[1], 0.5f * 180 / PI);
    near_value(channels[2], -179);
    near_value(channels[3], 179);
    assert(channels[4] == 20000 && channels[5] == 1);
    near_value(channels[6], 0.5f * 180 / PI);
    near_value(channels[7], -0.25f * 180 / PI);
    near_value(channels[8], 0.012f);
    near_value(channels[9], 0.047f / 3.0f);
    near_value(channels[10], 0.008f);
    near_value(channels[11], 0.043f / 3.0f);
#if TUNING_DIAGNOSTICS_ENABLED && TUNING_VOFA_TX_ENABLED
    near_value(channels[12], 90.0f);
    assert(channels[13] == 2 && channels[14] == 1);
    assert(channels[15] == 6000 && channels[16] == -6100);
    for (unsigned axis = 0; axis < 3; ++axis) {
        near_value(channels[17 + axis], mock_gyro_mean[axis] * 57.2958f);
        near_value(channels[20 + axis], mock_gyro_std[axis] * 57.2958f);
    }
#else
    assert(gyro_stats_reads == 0);
#endif
    fire_gate_debug_t fire_after = fire_gate_debug;
    assert(memcmp(&fire_before, &fire_after, sizeof(fire_before)) == 0);
    assert(memcmp(&YawCtrl, &yaw_before, sizeof(YawCtrl)) == 0);
    assert(memcmp(&PitchCtrl, &pitch_before, sizeof(PitchCtrl)) == 0);
#if TUNING_DIAGNOSTICS_ENABLED && TUNING_VOFA_TX_ENABLED
    /* 原始拨杆1/3/2、摩擦轮开关和编码器变化都独立上传。 */
    const uint8_t switches[] = {1, 3, 2};
    for (unsigned i = 0; i < 3; ++i) {
        rc.rc.s[1] = switches[i];
        mock_friction_enabled = i != 1;
        snapshot();
        assert(channels[13] == switches[i]);
        assert(channels[14] == mock_friction_enabled);
    }
    pitch_ecd = 0;
    snapshot();
    near_value(channels[12], -180.0f);
    pitch_ecd = 6144;
    mock_gyro_mean[0] = NAN;
    mock_gyro_std[2] = INFINITY;
    snapshot();
    assert(channels[17] == 0 && channels[22] == 0);
    assert(isnan(mock_gyro_mean[0]) && isinf(mock_gyro_std[2]));
    mock_gyro_mean[0] = 2.0f;
    mock_gyro_std[2] = 1.0f;
#endif
    irq_key = 1;
    snapshot();
    assert(irq_key == 1);
    irq_key = 0;
    YawCtrl.Volt_out = 0;
    snapshot();
    assert(channels[4] == 0);
    pitch_control_debug.actual_angle_rad = INFINITY;
    snapshot();
    assert(channels[0] == 0);
    pitch_control_debug.actual_angle_rad = -0.25f;
    YawCtrl.radspeed = INFINITY;
    snapshot();
    assert(channels[7] == 0);
    YawCtrl.radspeed = -0.00025f;
    /* 门控前置条件不满足时上传清零记录，不能保留上一帧的误差/阈值。 */
    fire_gate_debug = (fire_gate_debug_t){0};
    snapshot();
    for (unsigned i = 8; i < 12; ++i)
        assert(channels[i] == 0);
    fire_gate_debug = fire_before;
    fire_gate_debug.yaw_error_rad = NAN;
    fire_gate_debug.pitch_threshold_rad = INFINITY;
    snapshot();
    assert(channels[8] == 0 && channels[11] == 0);
    assert(isnan(fire_gate_debug.yaw_error_rad) && isinf(fire_gate_debug.pitch_threshold_rad));
    fire_gate_debug = fire_before;
    /* CAN超时不依赖自瞄模式，串口模拟也不能刷新CAN在线状态。 */
    now_ms = 1334;
    snapshot();
    assert(channels[5] == 1);
    now_ms = 1335;
    snapshot();
    assert(channels[5] == 0);
    NX_SelectSource(NX_SOURCE_UART);
    assert(NX_ReceiveFrame(can_payload, NX_SOURCE_UART));
    snapshot();
    assert(channels[5] == 0);
    assert(!NX_ReceiveFrame(can_payload, NX_SOURCE_CAN));
    snapshot();
    assert(channels[5] == 1); /* 即便未选CAN控制，仍能检测真实CAN收包。 */
    NX_SelectSource(NX_SOURCE_CAN);
    now_ms = UINT32_MAX - 20U;
    assert(NX_ReceiveFrame(can_payload, NX_SOURCE_CAN));
    now_ms = 79;
    snapshot();
    assert(channels[5] == 1);
    now_ms = 80;
    snapshot();
    assert(channels[5] == 0);
    assert(NX_ReceiveFrame(can_payload, NX_SOURCE_CAN));
    snapshot();
    assert(channels[5] == 1 && irq_key == 0);
    if (argc > 1) {
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        assert(fwrite(frame, 1, sizeof(frame), stdout) == sizeof(frame));
    } else {
        printf("PASS: VOFA %u-channel codec/service + original 12 unchanged + diagnostic switch + read-only snapshots/nonfinite handling/CAN timeout\n", TUNING_CHANNEL_COUNT);
    }
    return 0;
}
'''
    (OUT / 'test.c').write_text(prefix + declarations + funcs + '\n#include "tuning_service.c"\n' + tests, encoding='utf-8')
    executable = OUT / ('test_tuning.exe' if os.name == 'nt' else 'test_tuning')
    cc = os.environ.get('HOST_CC') or shutil.which('gcc')
    assert cc, 'Host GCC required'
    for diagnostics in (0, 1):
        subprocess.run([cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-Wno-unused-variable',
                    f'-DTUNING_DIAGNOSTICS_ENABLED={diagnostics}',
                    '-I', str(OUT), '-I', str(ROOT / 'Control'), '-I', str(ROOT / 'Task'),
                    '-I', str(ROOT / 'Applications'), '-I', str(ROOT / 'Debug'),
                    str(OUT / 'test.c'), str(ROOT / 'Debug/tuning_protocol.c'),
                    '-lm', '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
        frame = subprocess.check_output([str(executable), 'frame'])
        count = 12
        if diagnostics:
            count = 23
        assert len(frame) == count * 4 + 4 and frame[-4:] == b'\x00\x00\x80\x7f'
        values = struct.unpack(f'<{count}f', frame[:-4])
        assert abs(values[0] + 0.25 * 180 / 3.1415926) < 0.0002
        assert abs(values[3] - 179) < 0.0002 and values[4] == 0 and values[5] == 1
        assert abs(values[6] - 0.5 * 180 / 3.1415926) < 0.0002
        assert abs(values[7] + 0.25 * 180 / 3.1415926) < 0.0002
        for actual, expected in zip(values[8:12], (0.012, 0.047 / 3, 0.008, 0.043 / 3)):
            assert abs(actual - expected) < 1e-7
        if diagnostics:
            assert values[:12] == baseline
            assert abs(values[12] - 90) < 0.0001
            assert values[13:17] == (2, 1, 6000, -6100)
            for actual, value in zip(values[17:], (2, -3, 0.5, 0.25, 0.5, 1)):
                assert abs(actual - value * 57.2958) < 0.00003
        else:
            baseline = values
        print(f'PASS: independently decoded {len(frame)}-byte, {count}-channel little-endian JustFloat frame')



if __name__ == '__main__':
    main()
