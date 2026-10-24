#ifndef SHOT_DETECTION_H
#define SHOT_DETECTION_H

#include <stdint.h>

/* 用原始rpm判断局部峰谷，VOFA显示增益和偏置不参与检测。 */
#define SHOT_DETECTION_DROP_RPM 200.0f
#define SHOT_DETECTION_RECOVER_RPM 150.0f // 从当前谷底回升此幅度后允许下一次检测
#define SHOT_DETECTION_STABLE_BAND_RPM 80.0f
#define SHOT_DETECTION_MIN_RPM 7000.0f
#define SHOT_DETECTION_STABLE_MS 100U
#define SHOT_DETECTION_FLAG_MS 50U

/* 只保存检测内部状态，不作为电机控制输入。 */
typedef struct
{
    float peak_rpm;   // 当前下降段的局部参考峰值，不要求回到开轮时的空载转速
    float trough_rpm; // 当前下凹的最低值，用于局部回升判定
    float stable_min_rpm;
    float stable_max_rpm;
    uint32_t stable_since_ms;
    uint8_t stabilizing;
    uint8_t ready;
    uint8_t armed;
} shot_detector_t;

/* 对外发布的事件快照：累计次数和时刻用于后续视觉读取，flag用于VOFA画图。 */
typedef struct
{
    uint32_t count;
    uint32_t last_event_ms; // 板端HAL毫秒时钟，允许回绕，不是接收方时间。
    uint8_t flag;          // 掉速触发即置1，保持50 ms；读取不清除事件。
} shoot_detection_state_t;

void shot_detector_reset(shot_detector_t *detector);
/* enabled仅用于排除关轮减速；触发不依赖拨弹、遥控或视觉开火指令。 */
uint8_t shot_detector_update(shot_detector_t *detector, float rpm,
                             uint8_t enabled, uint32_t now_ms);

#endif
