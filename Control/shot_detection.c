#include "shot_detection.h"
#include <math.h>

void shot_detector_reset(shot_detector_t *detector)
{
    detector->peak_rpm = 0.0f;
    detector->trough_rpm = 0.0f;
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
    // 波动超过稳定带宽，以当前样本重新开始完整稳定窗口。
    if (spread_rpm > SHOT_DETECTION_STABLE_BAND_RPM)
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
    detector->peak_rpm = (detector->stable_min_rpm + detector->stable_max_rpm) * 0.5f;
    detector->trough_rpm = detector->peak_rpm;
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
    // 首次建立稳态基线，准备期间不判发射。
    if (!detector->ready)
    {
        // 最低转速只用于启动准备，连发负载导致转速降低时不反复重置检测。
        if (rpm < SHOT_DETECTION_MIN_RPM)
        {
            shot_detector_reset(detector);
            return 0U;
        }
        prepareBaseline(detector, rpm, now_ms);
        return 0U;
    }

    // 已经报告过当前下凹，记录谷底并等待局部回升，不等待原空载转速。
    if (!detector->armed)
    {
        // 当前转速继续下降，只更新谷底，不能重复产生同一事件。
        if (rpm < detector->trough_rpm)
        {
            detector->trough_rpm = rpm;
        }
        float rebound_rpm = rpm - detector->trough_rpm;
        // 从谷底明显回升后，以回升后的转速为新起点，支持低位连续掉速。
        if (rebound_rpm >= SHOT_DETECTION_RECOVER_RPM)
        {
            detector->peak_rpm = rpm;
            detector->armed = 1U;
        }
        return 0U;
    }

    // 回升仍在继续时跟踪最新局部峰值，下一次下降与这个峰值比较。
    if (rpm > detector->peak_rpm)
    {
        detector->peak_rpm = rpm;
    }
    float drop_rpm = detector->peak_rpm - rpm;
    // 当前样本达到大幅掉速阈值就立即报告，不等回升，也不检查拨弹指令。
    if (drop_rpm >= SHOT_DETECTION_DROP_RPM)
    {
        detector->armed = 0U;
        detector->trough_rpm = rpm;
        return 1U;
    }
    // 小幅正常漂移缓慢跟随；明显下降时冻结参考峰值，避免把掉速吸收掉。
    if (drop_rpm <= SHOT_DETECTION_STABLE_BAND_RPM)
    {
        detector->peak_rpm += (rpm - detector->peak_rpm) * 0.05f;
    }
    return 0U;
}
