"""编译实际协议/服务和控制入口，验证错误包、参数原子性及来源切换；不连接硬件。"""
from pathlib import Path
import os
import re
import shutil
import struct
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import tuning_link as link

OUT = ROOT / 'build/host-tuning'
OUT.mkdir(parents=True, exist_ok=True)


def function(source, name):
    match = re.search(r'^(?:static )?(?:uint8_t|void) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


def build():
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
static tuning_message_t request, reply;
static void call(uint8_t command, uint16_t length, uint8_t expected)
{
    request.command = command; request.length = length; request.sequence = 17;
    tuning_service_request(&request, &reply);
    if (reply.payload[0] != expected)
        fprintf(stderr, "command=%u length=%u expected=%u actual=%u id=%u value=%f\n", command, length, expected, reply.payload[0], tuning_read_u16(request.payload + 1), tuning_read_float(request.payload + 3));
    assert(reply.command == (command | 0x80) && reply.sequence == 17);
    assert(reply.payload[0] == expected);
}
static void vision(uint8_t payload[8], float pitch)
{
    memset(payload, 0, 8); payload[0] = 1;
    tuning_write_u16(payload + 3, (uint16_t)(int16_t)roundf(-pitch * 32767 / PI));
}
static void test_protocol(void)
{
    tuning_parser_t parser = {0}; tuning_message_t decoded;
    uint8_t bytes[TUNING_FRAME_SIZE], bad[TUNING_FRAME_SIZE];
    assert(tuning_crc16((const uint8_t *)"123456789", 9) == 0x29b1);
    request.command = TUNING_PARAM_SET; request.sequence = 0x1234;
    request.length = TUNING_MAX_PAYLOAD;
    for (unsigned i = 0; i < request.length; ++i) request.payload[i] = (uint8_t)i;
    unsigned length = tuning_encode(&request, bytes);
    assert(length == TUNING_FRAME_SIZE);
    for (unsigned i = 0; i < length; ++i)
        assert(tuning_parser_feed(&parser, bytes[i], &decoded) == (i + 1 == length));
    assert(decoded.sequence == request.sequence && decoded.length == request.length);
    assert(memcmp(decoded.payload, request.payload, request.length) == 0);
    for (unsigned fault = 0; fault < 3; ++fault) {
        memcpy(bad, bytes, length);
        if (fault == 0) bad[length - 1] ^= 1;
        if (fault == 1) bad[2] = 99;
        if (fault == 2) bad[7] = 255;
        tuning_parser_reset(&parser);
        for (unsigned i = 0; i < length; ++i) assert(!tuning_parser_feed(&parser, bad[i], &decoded));
        for (unsigned i = 0; i < length; ++i)
            assert(tuning_parser_feed(&parser, bytes[i], &decoded) == (i + 1 == length));
    }
    assert(parser.rejected >= 3);
    tuning_parser_reset(&parser);
    assert(!tuning_parser_feed(&parser, 0x55, &decoded));
    assert(!tuning_parser_feed(&parser, 0xa5, &decoded));
    for (unsigned i = 0; i < length; ++i)
        assert(tuning_parser_feed(&parser, bytes[i], &decoded) == (i + 1 == length));
    request.length = TUNING_MAX_PAYLOAD + 1;
    assert(tuning_encode(&request, bytes) == 0);
}
static void test_service(void)
{
    call(TUNING_HELLO, 0, TUNING_OK); assert(reply.length == 41);
    assert(tuning_read_u16(reply.payload + 2) == 3);
    call(0x7f, 0, TUNING_BAD_COMMAND);
    call(TUNING_STATE, 1, TUNING_BAD_LENGTH);
    request.payload[0] = 2;
    tuning_write_u16(request.payload + 1, 1); tuning_write_float(request.payload + 3, -1200);
    tuning_write_u16(request.payload + 7, 2); tuning_write_float(request.payload + 9, 0.2f);
    call(TUNING_PARAM_SET, 13, TUNING_OK);
    assert(pitch_gravity_config.gain == -1200 && pitch_gravity_config.offset_rad == 0.2f);
    tuning_write_float(request.payload + 3, -1500);
    const float bad_values[] = {NAN, INFINITY, -INFINITY, 4.0f};
    for (unsigned i = 0; i < 4; ++i) {
        tuning_write_float(request.payload + 9, bad_values[i]);
        call(TUNING_PARAM_SET, 13, TUNING_OUT_OF_RANGE);
        assert(pitch_gravity_config.gain == -1200 && pitch_gravity_config.offset_rad == 0.2f);
    }
    tuning_write_u16(request.payload + 7, 1); call(TUNING_PARAM_SET, 13, TUNING_BAD_PARAMETER);
    tuning_write_u16(request.payload + 7, 999); call(TUNING_PARAM_SET, 13, TUNING_BAD_PARAMETER);
    request.payload[0] = 1; tuning_write_float(request.payload + 3, -PITCH_GRAVITY_MAX_OUT - 1);
    call(TUNING_PARAM_SET, 7, TUNING_OUT_OF_RANGE);
    tuning_write_float(request.payload + 3, -PITCH_GRAVITY_MAX_OUT); call(TUNING_PARAM_SET, 7, TUNING_OK);
    tuning_write_u16(request.payload, 1); call(TUNING_PARAM_GET, 2, TUNING_OK);
    assert(tuning_read_float(reply.payload + 3) == -PITCH_GRAVITY_MAX_OUT);
    tuning_write_u16(request.payload, 0); call(TUNING_PARAM_INFO, 2, TUNING_OK);
    assert(tuning_read_u16(reply.payload + 1) == 1 && reply.payload[3] == 0);
    tuning_write_u16(request.payload, 1); call(TUNING_PARAM_INFO, 2, TUNING_OK);
    assert(tuning_read_u16(reply.payload + 1) == 2 && reply.payload[3] == 1);
    tuning_write_u16(request.payload, 2); call(TUNING_PARAM_INFO, 2, TUNING_OK);
    assert(tuning_read_u16(reply.payload + 1) == 3 && reply.payload[3] == 2);
    assert(tuning_read_float(reply.payload + 4) == 0 && tuning_read_float(reply.payload + 8) == PITCH_SPD_KI_MAX);
    assert(strcmp((char *)reply.payload + 20, "pitch.speed.ki") == 0);
    tuning_write_u16(request.payload, 3); call(TUNING_PARAM_INFO, 2, TUNING_BAD_PARAMETER);

    /* Ki 单写不能改变重力参数或已有 I；无效 Ki/混合批次必须整体拒绝。 */
    PitchCtrl.spd_pid.Iout = 321;
    request.payload[0] = 1; tuning_write_u16(request.payload + 1, 3);
    tuning_write_float(request.payload + 3, 500); call(TUNING_PARAM_SET, 7, TUNING_OK);
    assert(PitchCtrl.spd_pid.Ki == 500 && PitchCtrl.spd_pid.Iout == 321);
    assert(pitch_gravity_config.gain == -PITCH_GRAVITY_MAX_OUT && pitch_gravity_config.offset_rad == 0.2f);
    tuning_write_u16(request.payload, 3); call(TUNING_PARAM_GET, 2, TUNING_OK);
    assert(tuning_read_float(reply.payload + 3) == 500);
    request.payload[0] = 3;
    tuning_write_u16(request.payload + 1, 1); tuning_write_float(request.payload + 3, 600);
    tuning_write_u16(request.payload + 7, 2); tuning_write_float(request.payload + 9, 0.1f);
    tuning_write_u16(request.payload + 13, 3);
    const float bad_ki[] = {-1, PITCH_SPD_KI_MAX + 1, NAN, INFINITY};
    for (unsigned i = 0; i < 4; ++i) {
        tuning_write_float(request.payload + 15, bad_ki[i]);
        call(TUNING_PARAM_SET, 19, TUNING_OUT_OF_RANGE);
        assert(PitchCtrl.spd_pid.Ki == 500 && PitchCtrl.spd_pid.Iout == 321);
        assert(pitch_gravity_config.gain == -PITCH_GRAVITY_MAX_OUT && pitch_gravity_config.offset_rad == 0.2f);
    }
    tuning_write_float(request.payload + 15, PITCH_SPD_KI_MAX);
    call(TUNING_PARAM_SET, 19, TUNING_OK);
    assert(PitchCtrl.spd_pid.Ki == PITCH_SPD_KI_MAX && PitchCtrl.spd_pid.Iout == 321);
    assert(pitch_gravity_config.gain == 600 && pitch_gravity_config.offset_rad == 0.1f);
    /* 关闭 Ki 必须去掉残留 I；临界区嵌套后恢复调用者原中断状态。 */
    irq_key = 1;
    assert(gimbal_tuning_set_parameters(600, 0.1f, 0) == TUNING_OK);
    assert(irq_key == 1 && PitchCtrl.spd_pid.Ki == 0 && PitchCtrl.spd_pid.Iout == 0);
    irq_key = 0;
    tuning_write_u16(request.payload, 50); call(TUNING_STREAM, 2, TUNING_OUT_OF_RANGE);
    assert(tuning_service_period_ms() == 0);
    tuning_write_u16(request.payload, 200); call(TUNING_STREAM, 2, TUNING_OK);
    assert(tuning_service_period_ms() == 200);
    pitch_control_debug.control_enabled = 1; pitch_control_debug.total_saturated = 1;
    pitch_control_debug.motor_cmd = -123; PitchCtrl.spd_pid.Ki = 0;
    call(TUNING_STATE, 0, TUNING_OK);
    assert(reply.length == 76 && tuning_read_u16(reply.payload + 8) == 17);
    assert((int16_t)tuning_read_u16(reply.payload + 70) == -123);
    assert(tuning_read_float(reply.payload + 72) == 0);
}
static void test_control(void)
{
    uint8_t payload[8]; vision(payload, 0.2f);
    assert(gimbal_tuning_set_source(NX_SOURCE_UART) == TUNING_NOT_ENABLED);
    rc.rc.s[1] = 2;
    rc_online = 0; assert(gimbal_tuning_set_source(1) == TUNING_NOT_ENABLED);
    rc_online = 1; mode = RobotState_e_Powerless;
    assert(gimbal_tuning_set_source(1) == TUNING_NOT_ENABLED);
    mode = RobotState_e_CommonCar;
    assert(gimbal_tuning_submit_vision(payload) == TUNING_WRONG_SOURCE);
    assert(gimbal_tuning_set_source(1) == TUNING_OK);
    assert(gimbal_tuning_set_source(2) == TUNING_BAD_PARAMETER);
    YawCtrl.want_AbsoluteAngleCtrl = 0.4f; PitchCtrl.nowAbsoluteAngle = -0.1f;
    assert(gimbal_tuning_submit_vision(payload) == TUNING_OK);
    uint8_t can_payload[8]; vision(can_payload, -0.2f); can_payload[5] = 1;
    assert(!NX_ReceiveFrame(can_payload, NX_SOURCE_CAN) && !fire_t);
    getControlAngles(); assert(fabsf(PitchCtrl.want_AbsoluteAngleCtrl - 0.2f) < 0.0001f);
    assert(YawCtrl.want_AbsoluteAngleCtrl == 0.4f && !data_ready);
    getControlAngles(); assert(fabsf(PitchCtrl.want_AbsoluteAngleCtrl - 0.2f) < 0.0001f);
    payload[5] = 1; assert(gimbal_tuning_submit_vision(payload) == TUNING_BAD_PARAMETER);
    vision(payload, 0.8f); assert(gimbal_tuning_submit_vision(payload) == TUNING_OUT_OF_RANGE);
    vision(payload, 0.2f); assert(gimbal_tuning_submit_vision(payload) == TUNING_OK);
    rc.rc.s[1] = 3; getControlAngles(); assert(NX_GetSource() == 0 && !data_ready);
    rc.rc.s[1] = 2; assert(gimbal_tuning_set_source(1) == TUNING_OK);
    assert(gimbal_tuning_submit_vision(payload) == TUNING_OK);
    rc_online = 0; getControlAngles(); assert(NX_GetSource() == 0 && !data_ready);
    assert(gimbal_tuning_submit_vision(payload) == TUNING_NOT_ENABLED);
    rc_online = 1;
    assert(NX_ReceiveFrame(can_payload, NX_SOURCE_CAN));
    getControlAngles(); assert(fabsf(PitchCtrl.want_AbsoluteAngleCtrl + 0.2f) < 0.0001f);
    assert(YawCtrl.want_AbsoluteAngleCtrl == 0 && fire_t == 1);
    /* 自瞄收到的 CAN 帧要同时记给开火门控。 */
    assert(fire_frame_valid && fire_frame_source == NX_SOURCE_CAN && fire_frame_fire == 1);
    assert(PitchCtrl.spd_pid.Ki == 0.0f);
    assert(irq_key == 0);
}
int main(int argc, char **argv)
{
    (void)argv;
    if (argc > 1) {
#ifdef _WIN32
        _setmode(_fileno(stdin), _O_BINARY); _setmode(_fileno(stdout), _O_BINARY);
#endif
        tuning_parser_t parser = {0}; uint8_t bytes[TUNING_FRAME_SIZE]; int byte;
        while ((byte = getchar()) != EOF) {
            if (tuning_parser_feed(&parser, (uint8_t)byte, &request)) {
                tuning_service_request(&request, &reply);
                unsigned size = tuning_encode(&reply, bytes);
                fwrite(bytes, 1, size, stdout); fflush(stdout);
            }
        }
        return 0;
    }
    test_protocol(); test_service(); test_control();
    puts("PASS: actual C codec/service/control entry; invalid frames/atomic writes/gates/source switching/Yaw preservation");
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
    return executable


class FirmwarePort:
    """通过真实 C 服务交换二进制帧，接收时故意切成三字节小包。"""
    def __init__(self, executable):
        self.process = subprocess.Popen([str(executable), '--bridge'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.pending = bytearray()

    @property
    def in_waiting(self):
        return min(3, len(self.pending))

    def write(self, data):
        self.process.stdin.write(data)
        self.process.stdin.flush()
        header = self.process.stdout.read(8)
        assert len(header) == 8
        size, = struct.unpack_from('<H', header, 6)
        self.pending.extend(header + self.process.stdout.read(size + 2))
        return len(data)

    def read(self, size):
        data = bytes(self.pending[:size])
        del self.pending[:size]
        return data

    def close(self):
        self.process.stdin.close()
        self.process.wait(timeout=5)
        self.process.stdout.close()


class HostTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.executable = build()

    def test_python_client_with_actual_c_service(self):
        client = link.TuningLink(FirmwarePort(self.executable))
        try:
            self.assertEqual(client.hello()['parameter_count'], 3)
            parameters = client.parameters()
            self.assertEqual([p['id'] for p in parameters], [1, 2, 3])
            self.assertEqual(parameters[2]['minimum'], 0)
            self.assertEqual(parameters[2]['maximum'], 8000)
            client.set_parameters({1: -1400, 2: 0.1})
            self.assertEqual(client.get_parameter(1), -1400)
            state = client.state()
            self.assertEqual(state['gain'], -1400)
            self.assertEqual(state['speed_ki'], 0)
            self.assertEqual(state['source'], 0)
            client.set_parameters({3: 500})
            self.assertEqual(client.get_parameter(3), 500)
            self.assertEqual(client.state()['speed_ki'], 500)
            self.assertEqual(client.state()['gain'], -1400)
            with self.assertRaises(ValueError):
                client.set_parameters({1: 1000, 2: 0.2, 3: -1})
            self.assertEqual(client.get_parameter(1), -1400)
            self.assertEqual(client.get_parameter(3), 500)
            client.set_parameters({3: 0})
            self.assertEqual(client.state()['i_out'], 0)
            self.assertEqual(state['tick_ms'], 1234)
            self.assertAlmostEqual(state['offset_rad'], 0.1)
            client.stream(200)
            with self.assertRaises(ValueError):
                client.set_parameters({1: -1500, 2: float('nan')})
            self.assertEqual(client.get_parameter(1), -1400)
            with self.assertRaises(ValueError):
                client.select_source(True)  # bridge remote has not entered automatic mode
        finally:
            client.close()

    def test_decoder_recovery_and_fragments(self):
        frame = link.encode(link.Message(link.PARAM_GET, 17, b'\x01\x00'))
        bad = bytearray(frame); bad[-1] ^= 1
        decoder = link.Decoder()
        stream = b'noise\xa5' + bad + frame * 2
        received = []
        for byte in stream:
            received.extend(decoder.feed(bytes([byte])))
        self.assertEqual(received, [link.Message(link.PARAM_GET, 17, b'\x01\x00')] * 2)
        self.assertGreater(decoder.rejected, 0)
        payload = link.vision_payload(0.2)
        self.assertEqual(payload[0], 1)
        self.assertAlmostEqual(-struct.unpack_from('<h', payload, 3)[0] * 3.1415926 / 32767, 0.2, places=4)
        for boundary in (-0.45, 0.7):
            raw, = struct.unpack_from('<h', link.vision_payload(boundary), 3)
            decoded = -raw * 3.1415926 / 32767
            self.assertGreaterEqual(decoded, -0.45)
            self.assertLessEqual(decoded, 0.7)

    def test_failed_write_and_timeout_are_not_success(self):
        class Port:
            in_waiting = 0
            short_write = True
            def write(self, data):
                return len(data) - int(self.short_write)
            def read(self, size):
                return b''
        port = Port()
        client = link.TuningLink(port)
        with self.assertRaises(IOError):
            client.request(link.STATE)
        port.short_write = False
        client.decoder.feed(b'\xa5\x5a\x01')
        with self.assertRaises(TimeoutError):
            client.request(link.STATE, timeout=0.001)
        self.assertFalse(client.decoder.buffer)

    def test_telemetry_demultiplexing(self):
        state = struct.pack('<BIBBBH15fhf', 0, 123, 1, 1, 1, 17, *range(15), -100, 0)
        class Port:
            in_waiting = 512
            def write(self, data):
                request = link.Decoder().feed(data)[0]
                self.data = link.encode(link.Message(link.TELEMETRY, 0, state))
                self.data += link.encode(link.Message(request.command | 128, request.sequence - 1, b'\0old'))
                self.data += link.encode(link.Message(request.command | 128, request.sequence, state))
                return len(data)
            def read(self, size):
                data, self.data = self.data[:size], self.data[size:]
                return data
        client = link.TuningLink(Port())
        self.assertEqual(client.state()['motor_cmd'], -100)
        self.assertEqual(len(client.telemetry), 1)
        self.assertTrue(client.telemetry[0]['total_saturated'])


if __name__ == '__main__':
    unittest.main()
