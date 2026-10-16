"""只读执行现有视觉编码/发送函数，验证板端绝对角与双环 PID 适配。

python tools/check_vision_absolute_adapter.py --vision <视觉目录> --gcc gcc
依赖 numpy、主机 GCC。仅使用内存报文和模拟反馈，不启动 ROS，不连接 CAN。
所有测试输出写入电控 build/absolute_adapter_check，视觉文件不会被导入或写入。
"""
from pathlib import Path
from types import SimpleNamespace
import argparse
import ast
import hashlib
import json
import math
import re
import struct
import subprocess
import time

import numpy as np

BOARD = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--vision', type=Path, required=True)
parser.add_argument('--gcc', default='gcc')
parser.add_argument('--report', default='results.json')
args = parser.parse_args()
VISION = args.vision.resolve()
OUT = BOARD / 'build/absolute_adapter_check'
OUT.mkdir(parents=True, exist_ok=True)
hashes = {}


def read(path):
    data = path.read_bytes()
    hashes[str(path)] = hashlib.sha256(data).hexdigest()
    return data.decode('utf-8-sig').replace('\r\n', '\n')


def execute(nodes, env, filename):
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), filename, 'exec'), env)


def load_class(relative, name, env, methods):
    tree = ast.parse(read(VISION / relative))
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)
    node.bases = []
    node.body = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    assert len(node.body) == len(methods)
    execute([node], env, relative)
    return env[name]


signs = {}
for node in ast.parse(read(VISION / 'tasks/config/config.py')).body:
    if isinstance(node, ast.Assign):
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in ('CAN_YAW_SIGN', 'CAN_PITCH_SIGN'):
                signs[target.id] = ast.literal_eval(node.value)
ns = dict(np=np, math=math, time=time, struct=struct, RAD_FULL_SCALE=np.pi,
          cfg=SimpleNamespace(**signs), MpcControl=SimpleNamespace)
sender_cls = load_class('tasks/mpc/mpc_can_sender.py', 'MpcCanSender', ns, {'create_can_frame', 'control_callback'})
controller_cls = load_class('tasks/tracker/gimbal_controller.py', 'GimbalController', ns, {'encode_angle_for_mcu'})
tracker_cls = load_class('tasks/tracker/TrackerNode.py', 'TrackerNode', ns, {'publish_mpc_speed'})
selector_cls = load_class('tasks/tracker/armor_selector.py', 'ArmorSelector', ns, {'__init__', 'compute_gimbal_aim_fast'})
exec(compile(read(VISION / 'tasks/tracker/transform_utils.py'), 'transform_utils.py', 'exec'), ns)
tracker_tree = ast.parse(read(VISION / 'tasks/tracker/TrackerNode.py'))


def feedback_to_visual(yaw, pitch):
    env = {'msg': SimpleNamespace(gimbal_yaw=yaw, gimbal_pitch=pitch)}
    for name in ('gimbal_yaw', 'gimbal_pitch'):
        node = next(n for n in ast.walk(tracker_tree) if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == name for t in n.targets)
                    and any(isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name)
                            and t.value.id == 'msg' for t in ast.walk(n.value)))
        execute([node], env, 'TrackerNode.feedback')
    return env['gimbal_yaw'], env['gimbal_pitch']


def send_visual_target(yaw, pitch):
    # 保留视觉自己的 int16 编码、ROS 字段和取反操作，不在测试中改写其协议。
    encoded = controller_cls().encode_angle_for_mcu(yaw, pitch)
    messages = []
    tracker = tracker_cls()
    tracker.pub_mpc_control = SimpleNamespace(publish=messages.append)
    tracker.publish_mpc_speed(*encoded, fire=False)
    frames, logs = [], []
    sender = sender_cls()
    sender.can_id = 5
    sender.sock = SimpleNamespace(send=frames.append)
    sender.get_logger = lambda: SimpleNamespace(info=logs.append, error=logs.append)
    sender.control_callback(messages[0])
    assert len(frames) == 1, logs
    can_id, dlc, payload = struct.unpack('=IB3x8s', frames[0])
    assert can_id == 5 and dlc == 8
    return payload, logs


cases = []


def add(name, visual_target, expected, feedback=(0, 0), speed=(0, 0)):
    payload, logs = send_visual_target(*visual_target)
    raw = struct.unpack('<BhhBh', payload)
    cases.append(dict(name=name, visual_target=visual_target, expected=expected,
                      feedback=feedback, speed=speed, raw_yaw=raw[1], raw_pitch=raw[2],
                      payload_hex=payload.hex(' '), actual_vision_log=logs))


for yaw in (3.0, -3.0, 0.03, -0.03, 0.0):
    add(f'yaw {yaw:+g}', (yaw, 0), (yaw, 0))
for pitch in (0.2, -0.2, 0.6, -0.4):
    add(f'INS pitch {pitch:+g}', (0, -pitch), (0, pitch))
for pose in ((0.5, 0.2), (-0.5, -0.2), (3.1, 0.6), (-3.1, -0.4)):
    visual = feedback_to_visual(*pose)
    rotation, translation = ns['compute_cam_to_chassis_transform'](*visual)
    point_on_axis = rotation @ np.array([0.0, 0.0, 5.0]) + translation
    target = selector_cls().compute_gimbal_aim_fast(point_on_axis, *visual)
    add(f'optical-axis hold {pose}', target, pose, pose)
for target, current in ((-176, 179), (176, -179)):
    add(f'valid yaw target across pi {current} -> {target}', (math.radians(target), 0),
        (math.radians(target), 0), (math.radians(current), 0))
for pitch, limited in ((1.0, 0.7), (-1.0, -0.45)):
    add(f'board pitch limit {pitch:+g}', (0, -pitch), (0, limited))
for speed in ((0.5, -0.5), (-0.5, 0.5)):
    add(f'inner loop feedback {speed}', (0.03, -0.02), (0.03, 0.02), speed=speed)

# 复用电控既有测试的源码提取和 HAL 桩，停止在其 main 测试之前。
helper = BOARD / 'tools/check_vision_control.py'
env = {'__file__': str(helper), '__name__': 'adapter_source'}
found = False
for node in ast.parse(read(helper)).body:
    if isinstance(node, ast.AugAssign) and isinstance(node.value, ast.Constant) and 'int main(void)' in str(node.value.value):
        found = True
        break
    execute([node], env, str(helper))
assert found
source = env['source']
gimbal = read(BOARD / 'Task/gimbal_task.c')
for relative in ('Task/NX_communicate.c', 'Task/NX_communicate.h', 'Applications/CAN_recieve_send.c', 'Control/pid.c'):
    read(BOARD / relative)
for line in gimbal.splitlines():
    if re.match(r'#define (PITCH|YAW)_(SPD_K|AGL_K|VOLT_MAX_|AGL_SPD_MAX_)', line):
        source += line + '\n'
source += r'''
static void check_case(int n, const uint8_t bytes[8], float yaw, float pitch, float yaw_speed, float pitch_speed) {
    memset(&YawCtrl, 0, sizeof(YawCtrl)); memset(&PitchCtrl, 0, sizeof(PitchCtrl));
    NX_data = (CAN_Send_to_MCU){0}; vision_debug = (VisionDebug_t){0}; data_ready = 0;
    current_mode = lastMode = RobotState_e_CommonCar; auto_mode = 1;
    YawCtrl.nowAbsoluteAngle = yaw; PitchCtrl.nowAbsoluteAngle = pitch;
    YawCtrl.radspeed = yaw_speed; PitchCtrl.radspeed = pitch_speed;
    YawCtrl.want_AbsoluteAngleCtrl = 0.6f; PitchCtrl.want_AbsoluteAngleCtrl = -0.1f;
'''
for axis, prefix in (('Yaw', 'YAW'), ('Pitch', 'PITCH')):
    for loop, coeff, limit in (('agl', 'AGL', 'AGL_SPD'), ('spd', 'SPD', 'VOLT')):
        for field, macro in [('Kp', f'{coeff}_KP'), ('Ki', f'{coeff}_KI'), ('Kd', f'{coeff}_KD'),
                             ('max_out', f'{limit}_MAX_OUT'), ('max_iout', f'{limit}_MAX_IOUT')]:
            source += f'    {axis}Ctrl.{loop}_pid.{field} = {prefix}_{macro};\n'
        source += f'    {axis}Ctrl.{loop}_pid.mode = PID_POSITION;\n'
source += r'''
    receive(bytes); getControlAngles(); unitPID_calc();
    float target_y = YawCtrl.want_AbsoluteAngleCtrl, target_p = PitchCtrl.want_AbsoluteAngleCtrl;
    for (int i = 0; i < 100; ++i) { getControlAngles(); unitPID_calc(); }
    assert(vision_debug.applied_count == 1);
    assert(YawCtrl.want_AbsoluteAngleCtrl == target_y && PitchCtrl.want_AbsoluteAngleCtrl == target_p);
    for (int i = 0; i < 100; ++i) { receive(bytes); getControlAngles(); unitPID_calc(); }
    assert(vision_debug.applied_count == 101);
    assert(YawCtrl.want_AbsoluteAngleCtrl == target_y && PitchCtrl.want_AbsoluteAngleCtrl == target_p);
    assert(fabsf(YawCtrl.spd_pid.set - YawCtrl.agl_pid.out) < 1e-6f);
    assert(fabsf(PitchCtrl.spd_pid.set - PitchCtrl.agl_pid.out) < 1e-6f);
    assert(fabsf(YawCtrl.spd_pid.fdb - yaw_speed) < 1e-6f);
    assert(fabsf(PitchCtrl.spd_pid.fdb - pitch_speed) < 1e-6f);
    assert(YawCtrl.Volt_out == (int16_t)YawCtrl.spd_pid.out);
    assert(PitchCtrl.Volt_out == (int16_t)-PitchCtrl.spd_pid.out);
    printf("%d %.9f %.9f %.9f %.9f %d %d\n", n, target_y, target_p,
           YawCtrl.agl_pid.error[0], PitchCtrl.agl_pid.error[0], YawCtrl.Volt_out, PitchCtrl.Volt_out);
}
int main(void) {
'''
for i, case in enumerate(cases):
    data = ','.join('0x' + b for b in case['payload_hex'].split())
    values = ','.join(f'{v:.12f}f' for v in (*case['feedback'], *case['speed']))
    source += f'check_case({i}, (uint8_t[8]){{{data}}}, {values});\n'
source += 'return 0;\n}\n'
c_file, exe = OUT / 'check.c', OUT / 'check.exe'
c_file.write_text(source, encoding='utf-8')
subprocess.run([args.gcc, '-std=c99', '-O2', '-I', str(BOARD / 'Applications'),
                '-I', str(BOARD / 'Task'), '-I', str(BOARD / 'Control'), str(c_file),
                '-o', str(exe), '-lm'], check=True)
output = subprocess.check_output([str(exe)], text=True)
assert len(output.splitlines()) == len(cases)
failed = 0
for line in output.splitlines():
    i, yaw, pitch, err_y, err_p, motor_y, motor_p = line.split()
    case = cases[int(i)]
    targets = float(yaw), float(pitch)
    errors = float(err_y), float(err_p)
    ok = all(abs(actual - expected) < 1e-4 for actual, expected in zip(targets, case['expected']))
    ok &= all(abs(actual - math.remainder(expected - fb, 2 * math.pi)) < 1e-4
              for actual, expected, fb in zip(errors, case['expected'], case['feedback']))
    case['mcu'] = dict(target=targets, error=errors, simulated_motor_output=(int(motor_y), int(motor_p)), passed=ok)
    failed += int(not ok)
    print(f'{"PASS" if ok else "FAIL"} {case["name"]}: target=({yaw}, {pitch})')
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == digest for p, digest in hashes.items())
report = dict(scope='Original vision source read only; actual encode/send and MCU dual PID functions with mocked feedback/socket.',
              vision_root=str(VISION), cases=cases, failed=failed, source_sha256=hashes,
              note='No hardware response or ROS runtime validation; repeated frames and hold cycles each tested 100 times.')
(OUT / args.report).write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(f'{len(cases) - failed}/{len(cases)} passed; vision source unchanged; report: {OUT / args.report}')
raise SystemExit(1 if failed else 0)
