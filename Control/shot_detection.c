#include "shot_detection.h"
#include <math.h>

void shot_detector_reset(shot_detector_t *detector)
{
    detector->baseline_rpm = 0.0f;
    detector->stable_min_rpm = 0.0f;
    detector->stable_max_rpm = 0.0f;
    detector->stable_since_ms = 0U;
    detector->stabilizing = 0U;
    detector->ready = 0U;
    detector->armed = 0U;
}

static void startStableWindow(shot_detector_t *detector, float rpm, uint32_t now_ms)
{
    detector->stable_min_rpm = rpm;
    detector->stable_max_rpm = rpm;
    detector->stable_since_ms = now_ms;
    detector->stabilizing = 1U;
}

/* 开轮加速完成后，在100 ms内保持窄幅波动才建立基线，避免启动过程误触发。 */
static void prepareBaseline(shot_detector_t *detector, float rpm, uint32_t now_ms)
{
    // 新窗口或转速离开稳定带时重新开始，不能把持续加速当成稳态。
    if (!detector->stabilizing)
    {
        startStableWindow(detector, rpm, now_ms);
    }
    // 保存窗口内最低转速，用于判断实际波动范围。
    if (rpm < detector->stable_min_rpm)
    {
        detector->stable_min_rpm = rpm;
    }
    // 保存窗口内最高转速，用于判断实际波动范围。
    if (rpm > detector->stable_max_rpm)
    {
        detector->stable_max_rpm = rpm;
    }
    float spread_rpm = detector->stable_max_rpm - detector->stable_min_rpm;
    // 波动超过恢复带宽，以当前样本重新开始完整稳定窗口。
    if (spread_rpm > SHOT_DETECTION_RECOVER_RPM)
    {
        startStableWindow(detector, rpm, now_ms);
        return;
    }
    uint32_t elapsed_ms = now_ms - detector->stable_since_ms;
    // 稳定时间不足时只观察，不产生发射事件。
    if (elapsed_ms < SHOT_DETECTION_STABLE_MS)
    {
        return;
    }
    detector->baseline_rpm = (detector->stable_min_rpm + detector->stable_max_rpm) * 0.5f;
    detector->ready = 1U;
    detector->armed = 1U;
}

uint8_t shot_detector_update(shot_detector_t *detector, float rpm,
                             uint8_t enabled, uint32_t now_ms)
{
    // 关轮后的自然减速不代表发射；只复位检测准备，不改累计事件。
    if (!enabled)
    {
        shot_detector_reset(detector);
        return 0U;
    }
    // 非有限读数不建立基线，也不能留下上一次已武装状态。
    if (!isfinite(rpm))
    {
        shot_detector_reset(detector);
        return 0U;
    }
    // 未达到工作转速时，继续等待开轮加速结束。
    if (rpm < SHOT_DETECTION_MIN_RPM)
    {
        shot_detector_reset(detector);
        return 0U;
    }
    // 首次建立稳态基线，准备期间不判发射。
    if (!detector->ready)
    {
        prepareBaseline(detector, rpm, now_ms);
        return 0U;
    }

    float drop_rpm = detector->baseline_rpm - rpm;
    // 同一次下凹只触发一次，必须回升到原基线附近后才允许下一次。
    if (!detector->armed)
    {
        // 转速已经回到基线附近，恢复下一次检测资格。
        if (drop_rpm <= SHOT_DETECTION_RECOVER_RPM)
        {
            detector->armed = 1U;
        }
        return 0U;
    }
    // 当前样本达到大幅掉速阈值就立即报告，不等回升，也不检查拨弹指令。
    if (drop_rpm >= SHOT_DETECTION_DROP_RPM)
    {
        detector->armed = 0U;
        return 1U;
    }
    // 只在基线附近缓慢跟随正常漂移，下降过程冻结基线，避免把事件吸收掉。
    if (fabsf(drop_rpm) <= SHOT_DETECTION_RECOVER_RPM)
    {
        detector->baseline_rpm += (rpm - detector->baseline_rpm) * 0.05f;
    }
    return 0U;
}
