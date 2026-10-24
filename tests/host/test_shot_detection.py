"""编译真实掉速检测及发射任务发布接口；不连接硬件。"""
from pathlib import Path
import os
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'build/host-shot-detection'


def function(source, name):
    match = re.search(r'^(?:static )?(?:float|void) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    source = (ROOT / 'Task/shoot_task.c').read_text(encoding='utf-8-sig')
    # 检测调用在5 ms任务中，无需VOFA在线，也不依赖拨弹模式。
    task = function(source, 'shoot_task')
    assert task.index('getShootMotorSpeed();') < task.index('updateShotDetection();')
    assert task.index('updateShotDetection();') < task.index('calcCurrent();')
    prefix = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "shot_detection.h"
typedef struct { int16_t speed_rpm; } motor_measure_t;
static motor_measure_t motors[2];
static uint32_t irq_key, now;
static uint8_t fricOn;
static shot_detector_t shotDetector;
static shoot_detection_state_t shotDetection;
static uint32_t __get_PRIMASK(void) { return irq_key; }
static void __disable_irq(void) { irq_key = 1; }
static void __set_PRIMASK(uint32_t key) { irq_key = key; }
static uint32_t HAL_GetTick(void) { return now; }
static const motor_measure_t *get_shoot_motor_measure_point(uint8_t motor)
{
    assert(irq_key == 1);
    return &motors[motor];
}
'''
    functions = '\n'.join(function(source, name) for name in (
        'getFrictionMeanRpm', 'updateShotDetection', 'shoot_get_detection'))
    tests = r'''
static void sample(uint32_t ms, int16_t rpm)
{
    now = ms;
    motors[0].speed_rpm = rpm;
    motors[1].speed_rpm = -rpm;
    updateShotDetection();
}
static void warm(uint32_t start, int16_t rpm)
{
    for (unsigned ms = 0; ms <= 100; ms += 5) sample(start + ms, rpm);
}
int main(int argc, char **argv)
{
    (void)argv;
    /* 可选离线输入：每行rpm、enabled、毫秒时刻，输出触发的数据行。 */
    if (argc > 1) {
        unsigned enabled, ms, row = 0;
        int rpm;
        while (scanf("%d %u %u", &rpm, &enabled, &ms) == 3) {
            uint32_t previous = shotDetection.count;
            fricOn = enabled;
            sample(ms, rpm);
            ++row;
            if (shotDetection.count != previous) printf("%u\n", row);
        }
        return 0;
    }
    fricOn = 1;
    for (unsigned ms = 0; ms < 200; ms += 5) sample(ms, ms * 40);
    assert(shotDetection.count == 0); /* 启动加速不计弹。 */
    warm(200, 8400);
    assert(shotDetector.ready && shotDetection.count == 0);
    sample(305, 8380);
    sample(310, 8410);
    sample(315, 8300);
    assert(shotDetection.count == 0); /* 空拨级的小波动不触发。 */
    sample(320, 8000);
    assert(shotDetection.count == 1 && shotDetection.flag == 1);
    assert(shotDetection.last_event_ms == 320); /* 下降这一轮就触发，不等回升。 */
    sample(325, 7900);
    sample(365, 8000);
    assert(shotDetection.count == 1 && shotDetection.flag == 1);
    sample(370, 8000);
    assert(shotDetection.count == 1 && shotDetection.flag == 0);
    sample(375, 8200);
    assert(shotDetection.count == 1); /* 没恢复前不能重复触发。 */
    sample(380, 8400);
    sample(385, 8100);
    assert(shotDetection.count == 2);
    shoot_detection_state_t snapshot;
    shoot_detection_state_t before = shotDetection;
    shoot_get_detection(&snapshot);
    assert(snapshot.count == 2 && snapshot.flag == 1 && snapshot.last_event_ms == 385);
    assert(irq_key == 0 && memcmp(&before, &shotDetection, sizeof(before)) == 0);
    irq_key = 1;
    shoot_get_detection(&snapshot);
    assert(irq_key == 1); /* 嵌套快照正确恢复中断状态，读事件不消费。 */
    irq_key = 0;
    fricOn = 0;
    for (unsigned ms = 400; ms < 500; ms += 5) sample(ms, 8400 - (ms - 400) * 80);
    assert(shotDetection.count == 2 && !shotDetector.ready && !shotDetection.flag);
    fricOn = 1;
    warm(500, 8400);
    sample(605, 8100);
    assert(shotDetection.count == 3); /* 重新开轮不清累计事件。 */
    motors[0].speed_rpm = -32768;
    motors[1].speed_rpm = 32767;
    assert(getFrictionMeanRpm() == 32767.5f); /* 绝对值不发生int16溢出。 */

    shot_detector_t detector = {0};
    for (unsigned ms = 0; ms <= 100; ms += 5)
        assert(!shot_detector_update(&detector, 8400, 1, ms));
    assert(shot_detector_update(&detector, 8200, 1, 105)); /* 等于200 rpm也触发。 */
    assert(!shot_detector_update(&detector, NAN, 1, 110) && !detector.ready);
    assert(!shot_detector_update(&detector, INFINITY, 1, 115));
    assert(!shot_detector_update(&detector, 6000, 1, 120));
    /* 稳定计时和50 ms显示脉冲均跨uint32回绕。 */
    shot_detector_reset(&shotDetector);
    memset(&shotDetection, 0, sizeof(shotDetection));
    warm(UINT32_MAX - 149U, 8400);
    sample(UINT32_MAX - 19U, 8000);
    sample(29U, 8000);
    assert(shotDetection.flag == 1 && shotDetection.count == 1);
    sample(30U, 8000);
    assert(shotDetection.flag == 0 && shotDetection.count == 1);
    shot_detector_reset(&detector);
    uint32_t start = UINT32_MAX - 49U;
    for (unsigned ms = 0; ms <= 100; ms += 5)
        assert(!shot_detector_update(&detector, 8400, 1, start + ms));
    assert(detector.ready && shot_detector_update(&detector, 8100, 1, start + 105U));
    puts("PASS: raw wheel RPM / startup and shutdown / immediate drop event / recovery rearm / no duplicate / 50ms flag / cumulative count / read-only atomic snapshot / invalid samples / integer and time wrap");
}
'''
    (OUT / 'test.c').write_text(prefix + functions + tests, encoding='utf-8')
    cc = os.environ.get('HOST_CC') or shutil.which('gcc')
    assert cc, 'Host GCC required'
    exe = OUT / ('test_shot_detection.exe' if os.name == 'nt' else 'test_shot_detection')
    subprocess.run([cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I', str(ROOT / 'Control'), str(OUT / 'test.c'),
                    str(ROOT / 'Control/shot_detection.c'), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
