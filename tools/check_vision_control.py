"""离线检查真实视觉解码/目标更新函数；模拟中断时序，不连接板子。

运行：rtk proxy python tools/check_vision_control.py
需要主机 GCC；测试文件和可执行文件放在临时目录。
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def block(text, marker):
    """提取完整函数/代码块，让测试直接执行工程实现。"""
    start = text.index(marker)
    begin = text.index("{", start)
    depth = 0
    for index in range(begin, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
        if depth == 0:
            return text[start:index + 1]
    raise ValueError(marker)


gimbal = (ROOT / "Task/gimbal_task.c").read_text(encoding="utf-8-sig")
can = (ROOT / "Applications/CAN_recieve_send.c").read_text(encoding="utf-8-sig")
nx = (ROOT / "Task/NX_communicate.c").read_text(encoding="utf-8-sig")
pid = (ROOT / "Control/pid.c").read_text(encoding="utf-8-sig")
decode = block(can, "case 0x005:").split("{", 1)[1].rsplit("}", 1)[0]
decode = decode.replace("break;", "return;")

source = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "NX_communicate.h"
#include "pid.h"
volatile CAN_Send_to_MCU NX_data;
volatile VisionDebug_t vision_debug;
volatile uint8_t data_ready;
uint8_t fire_t;
typedef struct { struct {int16_t ch[5]; uint8_t s[2];} rc; struct {int16_t x,y;} mouse; } RC;
RC rc; const RC *rc_p = &rc;
enum RobotState_e { RobotState_e_Powerless, RobotState_e_CommonCar };
enum RobotState_e current_mode = RobotState_e_CommonCar, lastMode = RobotState_e_CommonCar;
const enum RobotState_e *robotMode = &current_mode;
struct {
    float nowAbsoluteAngle, want_AbsoluteAngleCtrl, radspeed;
    uint8_t nopowerflag;
    pid_type_def agl_pid, spd_pid;
    int16_t Volt_out;
} PitchCtrl, YawCtrl;
float delta_pitch, delta_yaw;
int auto_mode = 1, inject_before_take, inject_on_restore;
uint32_t irq_mask;
int hcan2; const int *hcan = &hcan2;
struct { int IDE, RTR, DLC; } rx_header = {0, 0, 8};
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
const uint8_t positive[8] = {1, 0x3a, 0x7a, 0, 0, 0, 0, 0};
const uint8_t negative[8] = {1, 0xc6, 0x85, 0, 0, 0, 0, 0};
void receive(const uint8_t rx_data[8]) {
''' + decode + r'''
}
int robotIsAuto(void) {
    if (inject_before_take) { receive(negative); inject_before_take = 0; }
    return auto_mode;
}
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t value) {
    irq_mask = value;
    if (!value && inject_on_restore) { inject_on_restore = 0; receive(negative); }
}
#define PI 3.14159265358979323846f
#define RC_DEADLINE 10
#define PITCH_RC_SEN -0.000005f
#define PITCH_MOUSE_SEN -0.000005f
#define YAW_RC_SEN -0.000004f
#define YAW_MOUSE_SEN -0.000005f
#define PITCH_MAX_ANGLE 0.7f
#define PITCH_MIN_ANGLE -0.45f
#define rc_deadband_limit(input, output, dead) do {output = ((input) > (dead) || (input) < -(dead)) ? (input) : 0;} while (0)
#define LimitMax(value, limit) do {if ((value) > (limit)) (value) = (limit); else if ((value) < -(limit)) (value) = -(limit);} while (0)
#define range_limit_inside LimitMax
'''
for text, marker in (
    (nx, "uint8_t NX_TakeControlFrame(CAN_Send_to_MCU *frame)"),
    (gimbal, "void getControlAngles(void)\n{"),
    (gimbal, "fp32 radFormat(fp32 rawAngle)"),
    (gimbal, "fp32 gimbal_PID_calc(pid_type_def *pid, fp32 ref, fp32 set)"),
    (pid, "fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)"),
    (gimbal, "void unitPID_calc(void)\n{"),
):
    source += block(text, marker) + "\n"

source += r'''
int main(void) {
    YawCtrl.agl_pid.Kp = 0.006f; YawCtrl.agl_pid.max_out = 50.0f;
    YawCtrl.spd_pid.Kp = 10000000.0f; YawCtrl.spd_pid.max_out = 30000.0f;
    receive(positive); getControlAngles(); unitPID_calc();
    assert(NX_data.yaw == 31290 && fabsf(YawCtrl.want_AbsoluteAngleCtrl - 3.0f) < 0.0001f);
    assert(YawCtrl.Volt_out > 0);
    printf("+3: raw=%d target=%.6f output=%d\n", NX_data.yaw, YawCtrl.want_AbsoluteAngleCtrl, YawCtrl.Volt_out);
    receive(negative); getControlAngles(); unitPID_calc();
    assert(NX_data.yaw == -31290 && fabsf(YawCtrl.want_AbsoluteAngleCtrl + 3.0f) < 0.0001f);
    assert(YawCtrl.Volt_out < 0);
    printf("-3: raw=%d target=%.6f output=%d\n", NX_data.yaw, YawCtrl.want_AbsoluteAngleCtrl, YawCtrl.Volt_out);
    for (int i = 0; i < 100; ++i) {
        receive(positive); getControlAngles(); assert(YawCtrl.want_AbsoluteAngleCtrl > 0);
        receive(negative); getControlAngles(); assert(YawCtrl.want_AbsoluteAngleCtrl < 0);
    }
    receive(positive); getControlAngles();
    inject_before_take = 1; getControlAngles();
    assert(YawCtrl.want_AbsoluteAngleCtrl < 0 && data_ready == 0);
    puts("PASS: formerly lost frame now changes target");
    receive(positive); inject_on_restore = 1; getControlAngles();
    assert(YawCtrl.want_AbsoluteAngleCtrl > 0 && data_ready == 1);
    getControlAngles(); assert(YawCtrl.want_AbsoluteAngleCtrl < 0 && data_ready == 0);
    puts("PASS: frame arriving after snapshot remains pending for next cycle");
    receive(positive); irq_mask = 1; getControlAngles(); assert(irq_mask == 1); irq_mask = 0;
    uint32_t applied = vision_debug.applied_count;
    getControlAngles(); assert(vision_debug.applied_count == applied && YawCtrl.want_AbsoluteAngleCtrl > 0);
    auto_mode = 0; receive(negative); getControlAngles();
    assert(data_ready == 1 && vision_debug.applied_count == applied && vision_debug.auto_active == 0);
    auto_mode = 1; current_mode = RobotState_e_Powerless; getControlAngles();
    assert(data_ready == 1 && vision_debug.applied_count == applied && vision_debug.auto_active == 0);
    current_mode = RobotState_e_CommonCar; getControlAngles(); assert(YawCtrl.want_AbsoluteAngleCtrl < 0);
    uint32_t received = vision_debug.rx_count;
    rx_header.DLC = 4; receive(positive); assert(vision_debug.rx_count == received);
    rx_header.DLC = 8; rx_header.IDE = 1; receive(positive); assert(vision_debug.rx_count == received);
    rx_header.IDE = 0; rx_header.RTR = 1; receive(positive); assert(vision_debug.rx_count == received);
    assert(vision_debug.positive_count > 0 && vision_debug.negative_count > 0);
    assert(vision_debug.last_positive_raw == 31290 && vision_debug.last_negative_raw == -31290);
    puts("PASS: held target, manual/powerless gating, invalid frames, sign history, interrupt mask");
    puts("Offline checks passed; motor behavior still needs hardware observation.");
}
'''
with tempfile.TemporaryDirectory(prefix="bubingimbal-vision-check-") as directory:
    work = Path(directory)
    (work / "check.c").write_text(source, encoding="utf-8")
    subprocess.run([
        "gcc", "-std=c99", "-O2", "-I", str(ROOT / "Applications"),
        "-I", str(ROOT / "Task"), "-I", str(ROOT / "Control"),
        str(work / "check.c"), "-o", str(work / "check.exe"),
    ], check=True)
    subprocess.run([str(work / "check.exe")], check=True)
