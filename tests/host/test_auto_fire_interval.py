"""编译真实自瞄拨弹入口和状态机，验证当前配置的最小间隔；不连接硬件。"""
from pathlib import Path
import os
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/host-auto-fire-interval'


def function(source, name):
    match = re.search(r'^(?:static void|void static|void) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
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
static float wantedVShootMotor[2], wantedVTriggerMotor;
'''
    globals_ = '\nstatic enum TriggerMode_e triggerMode;\n'
    funcs = '\n'.join(function(source, name) for name in (
        'getInstruction', 'fricModeChange', 'tryStartAutoShot',
        'triggerModeChange', 'initShootModes', 'setSpeed'))
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
    step(40);
    assert(triggerOn && shootOneStartTime == 0);
    step(45);
    assert(!triggerOn); /* 41 ms持续时间保留，5 ms轮询在45 ms停止 */
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

    /* 手动单发、连发和反转仍沿用原逻辑，不受自瞄间隔限制。 */
    reset();
    automatic = 0;
    rc_data.rc.ch[4] = 100;
    step(0);
    assert(triggerOn);
    step(5);
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
    rc_data.rc.ch[4] = 0;
    step(125);
    rc_data.rc.ch[4] = -100;
    step(130);
    assert(triggerMode == TriggerMode_e_StuckSolve && wantedVTriggerMotor < 0);
    puts("PASS: configured auto fire interval / first shot / 41ms duration / no queue / mode and wheel toggles / mouse bypass / tick wrap / 24Hz and repeated frames / manual modes unchanged");
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
