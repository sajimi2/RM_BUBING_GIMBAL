"""编译真实拨弹状态机，验证自瞄间隔和手瞄连发脉冲上限；不连接硬件。"""
from pathlib import Path
import os
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/host-auto-fire-interval'


def function(source, name):
    match = re.search(r'^(?:static void|void static|void|static uint8_t|uint8_t) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    source = (ROOT / 'Task/shoot_task.c').read_text(encoding='utf-8-sig')
    timing = source[source.index('static TickType_t shootOneStartTime'):source.index('#define TRIGGER_ROUNDS')]
    enum_start = source.index('enum TriggerMode_e{')
    modes = source[enum_start:source.index('};', enum_start) + 2]
    prefix = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms)) /* 工程tick=1000 Hz */
#define NX_SOURCE_UART 1
static TickType_t tick;
static int automatic, gate, source;
static TickType_t xTaskGetTickCount(void) { return tick; }
static int robotIsAuto(void) { return automatic; }
static int gimbal_fire_allowed(void) { return gate; }
static int NX_GetSource(void) { return source; }
static struct { struct { int s[2], ch[5]; } rc;
                struct { int press_l, press_r; } mouse; } rc_data, *rc_p = &rc_data;
static struct { int rev; } nx_data, *nx_p = &nx_data;
static int trigger_state, wheel, fricOn, triggerOn;
static uint8_t fire_t;
static volatile uint8_t rc_online = 1;
static float wantedVShootMotor[2], wantedVTriggerMotor;
'''
    globals_ = '\nstatic enum TriggerMode_e triggerMode;\n'
    funcs = '\n'.join(function(source, name) for name in (
        'getInstruction', 'fricModeChange', 'tryStartAutoShot',
        'manualBurstRequested', 'updateManualBurst',
        'triggerModeChange', 'initShootModes', 'setSpeed', 'shoot_is_feeding_forward'))
    tests = r'''
static void step(TickType_t now)
{
    tick = now;
    getInstruction();
    fricModeChange();
    triggerModeChange();
    setSpeed();
}
static void reset(void)
{
    initShootModes();
    rc_online = 1;
    automatic = gate = source = 0;
    rc_data.mouse.press_l = rc_data.mouse.press_r = 0;
    rc_data.rc.ch[4] = 0;
    rc_data.rc.s[1] = 2;
    nx_data.rev = 0;
    step(0); /* 让原手动状态机的静态lastmode也回到Stop */
    automatic = gate = 1;
    fire_t = 0;
}
int main(void)
{
    const TickType_t interval = AUTO_FIRE_MIN_INTERVAL_TICKS;
    assert(interval >= 200U);
    /* 首发立即启动；0也能作为有效启动时刻。重复视觉帧不续期、不绕过间隔。 */
    reset();
    fire_t = 1;
    step(0);
    assert(triggerOn && autoFireHasStarted && autoFireLastStartTime == 0);
    assert(shoot_is_feeding_forward() == 1);
    rc_online = 0;
    assert(shoot_is_feeding_forward() == 0);
    assert(triggerOn && wantedVTriggerMotor > 0); /* 读取标志不修改控制状态。 */
    rc_online = 1;
    step(40);
    assert(triggerOn && shootOneStartTime == 0);
    step(45);
    assert(!triggerOn); /* 41 ms持续时间保留，5 ms轮询在45 ms停止 */
    assert(shoot_is_feeding_forward() == 0);
    for (unsigned now = 50; now < interval; now += 5) {
        fire_t = 1;
        step(now);
        assert(!triggerOn && autoFireLastStartTime == 0);
    }
    fire_t = 1;
    step(interval - 1);
    assert(!triggerOn);
    step(interval);
    assert(triggerOn && shootOneStartTime == interval && autoFireLastStartTime == interval);
    step(interval + 45);
    assert(!triggerOn);

    /* 到期仍须满足当下门控；失败请求不排队，也不消耗间隔。 */
    gate = 0;
    fire_t = 1;
    step(2 * interval);
    assert(!triggerOn && autoFireLastStartTime == interval);
    step(2 * interval + 50);
    assert(!triggerOn);
    gate = 1;
    step(2 * interval + 55);
    assert(triggerOn && autoFireLastStartTime == 2 * interval + 55);

    /* 开关摩擦轮、退出再进入自瞄，都不能清除冷却。 */
    reset();
    step(0);
    step(45);
    nx_data.rev = 1;
    step(50);
    assert(!fricOn && !triggerOn);
    assert(shoot_is_feeding_forward() == 0);
    nx_data.rev = 0;
    step(100);
    assert(!triggerOn);
    automatic = 0;
    step(150);
    automatic = 1;
    fire_t = 1;
    step(interval - 1);
    assert(!triggerOn);
    step(interval);
    assert(triggerOn);

    /* 自瞄中鼠标不能走手动连发旁路；切入自瞄也不能沿用手动连发。 */
    reset();
    gate = 0;
    rc_data.mouse.press_l = 1;
    step(0);
    assert(!triggerOn);
    triggerMode = TriggerMode_e_ShootMulti;
    rc_data.rc.ch[4] = 500;
    step(5);
    assert(!triggerOn && triggerMode == TriggerMode_e_Stop);
    gate = 1;
    step(10);
    assert(triggerOn);
    step(55);
    step(60);
    assert(!triggerOn);

    /* 无符号tick回绕：间隔到期前拦截，到期时放行。 */
    reset();
    TickType_t first = UINT32_MAX - 99U;
    step(first);
    assert(triggerOn);
    step(first + 45U);
    step(first + interval - 1);
    assert(!triggerOn);
    step(first + interval);
    assert(triggerOn && autoFireLastStartTime == (TickType_t)(first + interval));

    /* 真实收帧节奏24 Hz和最坏每5 ms重复请求，均须满足当前间隔。 */
    for (int every_poll = 0; every_poll <= 1; ++every_poll) {
        reset();
        unsigned frame = 0, starts = 0;
        TickType_t previous = 0;
        for (unsigned now = 0; now < 10000; now += 5) {
            if (every_poll || now * 24U >= frame * 1000U) {
                fire_t = 1;
                ++frame;
            }
            int was_active = triggerOn;
            step(now);
            if (triggerOn && !was_active) {
                if (starts) assert((TickType_t)(now - previous) >= interval);
                previous = now;
                ++starts;
            }
        }
        assert(starts == 9999U / interval + 1U);
    }

    /* 手动单发和反转不受自瞄间隔限制，手瞄连发只在脉冲内给正向目标。 */
    reset();
    automatic = 0;
    rc_data.rc.ch[4] = 100;
    step(0);
    assert(triggerOn);
    step(5);
    assert(shoot_is_feeding_forward() == 1);
    step(50);
    assert(!triggerOn);
    rc_data.rc.ch[4] = 0;
    step(55);
    rc_data.rc.ch[4] = 100;
    step(60);
    assert(triggerOn && !autoFireHasStarted);
    step(65);
    step(110);
    rc_data.rc.ch[4] = 500;
    step(115);
    step(120);
    assert(triggerMode == TriggerMode_e_ShootMulti && triggerOn);
    assert(wantedVTriggerMotor > 0);
    assert(shoot_is_feeding_forward() == 1);
    rc_data.rc.ch[4] = 0;
    step(125);
    rc_data.rc.ch[4] = -100;
    step(130);
    assert(triggerMode == TriggerMode_e_StuckSolve && wantedVTriggerMotor < 0);
    assert(triggerOn && shoot_is_feeding_forward() == 0);
    reset();
    source = NX_SOURCE_UART;
    step(0);
    assert(!fricOn && shoot_is_feeding_forward() == 0);
    /* 遥控连发：首发立即启动，单次41 ms，启动到启动至少间隔200 ms。 */
    const TickType_t manual_interval = MANUAL_BURST_MIN_INTERVAL_TICKS;
    assert(manual_interval == 200U);
    reset();
    automatic = gate = 0;
    rc_data.rc.ch[4] = 500;
    step(0);
    assert(triggerOn && manualBurstLastStartTime == 0);
    step(40);
    assert(triggerOn);
    step(45);
    assert(!triggerOn && wantedVTriggerMotor == 0);
    step(manual_interval - 1);
    assert(!triggerOn);
    step(manual_interval);
    assert(triggerOn && manualBurstLastStartTime == manual_interval);
    rc_data.rc.ch[4] = 0;
    step(manual_interval + 5);
    assert(!triggerOn); /* 松开在下一次5 ms轮询立即停，不等本脉冲结束。 */
    rc_data.rc.ch[4] = 500;
    step(manual_interval + 10);
    assert(!triggerOn); /* 快速松开再按，不续接刚才被打断的脉冲。 */
    step(2 * manual_interval);
    assert(triggerOn);
    rc_data.rc.s[1] = 3;
    step(2 * manual_interval + 5);
    assert(!triggerOn && !fricOn);
    rc_data.rc.s[1] = 2;
    step(2 * manual_interval + 10);
    assert(!triggerOn);
    automatic = 1;
    step(2 * manual_interval + 15);
    automatic = 0;
    step(2 * manual_interval + 20);
    assert(!triggerOn); /* 开关轮和切换模式均不重置手瞄连发间隔。 */
    step(3 * manual_interval);
    assert(triggerOn);

    /* 鼠标左键与拨轮共用限速；同时按下、反复点击都不能生成更密的连发脉冲。 */
    for (unsigned input = 0; input < 4; ++input) {
        reset();
        automatic = gate = 0;
        unsigned starts = 0;
        TickType_t previous = 0;
        for (unsigned now = 0; now < 2000; now += 5) {
            rc_data.rc.ch[4] = 0;
            rc_data.mouse.press_l = 0;
            if (input == 0 || input == 2) rc_data.rc.ch[4] = 500;
            if (input == 1 || input == 2) rc_data.mouse.press_l = 1;
            if (input == 3) rc_data.mouse.press_l = (now % 10 == 0);
            int was_active = triggerOn;
            step(now);
            if (triggerOn && !was_active) {
                if (starts) assert((TickType_t)(now - previous) >= manual_interval);
                previous = now;
                ++starts;
            }
        }
        assert(starts == 10);
    }
    /* 鼠标仍按住时，反转请求优先退出连发并进入原解堵逻辑。 */
    rc_data.rc.ch[4] = -100;
    step(2000);
    step(2005);
    assert(triggerMode == TriggerMode_e_StuckSolve && wantedVTriggerMotor < 0);

    reset();
    automatic = gate = 0;
    rc_data.rc.ch[4] = 500;
    TickType_t manual_first = UINT32_MAX - 99U;
    step(manual_first);
    assert(triggerOn);
    step(manual_first + 45U);
    step(manual_first + manual_interval - 1U);
    assert(!triggerOn);
    step(manual_first + manual_interval);
    assert(triggerOn); /* 手瞄间隔也支持tick回绕。 */
    step(manual_first + manual_interval + 1000U);
    assert(triggerOn && manualBurstLastStartTime == (TickType_t)(manual_first + manual_interval + 1000U));
    step(manual_first + manual_interval + 1005U);
    assert(triggerOn); /* 长时间未调度后只启动当前一发，不补发历史请求。 */
    puts("PASS: auto interval unchanged / manual wheel and mouse max 5 starts per second / 41ms pulses / release and repress / wheel and mode toggles / reverse priority / no queue / tick wrap / manual single unchanged");
}
'''
    (OUT / 'test.c').write_text(prefix + timing + modes + globals_ + funcs + tests, encoding='utf-8')
    cc = os.environ.get('HOST_CC') or shutil.which('gcc')
    assert cc, 'Host GCC required'
    exe = OUT / ('test_auto_fire_interval.exe' if os.name == 'nt' else 'test_auto_fire_interval')
    subprocess.run([cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-Wno-old-style-declaration', str(OUT / 'test.c'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
