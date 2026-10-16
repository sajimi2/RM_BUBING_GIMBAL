#include "vision_telemetry.h"
#include "NX_communicate.h"
#include "usart.h"
#include <stdio.h>

#if VISION_TELEMETRY_ENABLED && VISION_TELEMETRY_FORMAT == VISION_TELEMETRY_TEXT
/* 输出带符号的 rad，保留三位小数；避免嵌入式 printf 浮点支持和较大栈开销。 */
static void format_angle(char *text, uint16_t capacity, fp32 rad)
{
    if (!(rad >= -10000.0f && rad <= 10000.0f))
    {
        (void)snprintf(text, capacity, "invalid"); // 包含 NaN/Inf，不做越界整数转换。
        return;
    }
    const int32_t scaled = (int32_t)(rad * 1000.0f + (rad < 0.0f ? -0.5f : 0.5f));
    const int32_t magnitude = scaled < 0 ? -scaled : scaled;
    (void)snprintf(text, capacity, "%c%ld.%03ld", rad < 0.0f ? '-' : '+',
                   (long)(magnitude / 1000), (long)(magnitude % 1000));
}
#endif

void VisionTelemetry_Poll(void)
{
#if VISION_TELEMETRY_ENABLED
    /* DMA 完成前缓冲区必须保持有效且不被覆盖。USART6 专用于本诊断输出。 */
    static uint32_t last_sample_ms;
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_sample_ms) < VISION_TELEMETRY_PERIOD_MS)
        return;
    last_sample_ms = now;
    if (huart6.gState != HAL_UART_STATE_READY)
        return;

    /* 在短临界区内取得同一份观测快照；浮点转换和 DMA 调用在恢复中断后进行。 */
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const VisionDebug_t sample = vision_debug;
    __set_PRIMASK(interrupt_mask);

#if VISION_TELEMETRY_FORMAT == VISION_TELEMETRY_TEXT
    static char frame[256];
    static uint32_t sent_sequence;
    char target[16], feedback[16], error[16];
    format_angle(target, sizeof(target), sample.yaw_target_rad);
    format_angle(feedback, sizeof(feedback), sample.yaw_feedback_rad);
    format_angle(error, sizeof(error), sample.yaw_error_rad);
    const int length = snprintf(frame, sizeof(frame),
        "seq=%lu ms=%lu raw=%d used=%d target=%s fb=%s err=%s out=%d "
        "auto=%u rc=%u rx=%lu apply=%lu pos=%d neg=%d pc=%lu nc=%lu bus=%u\r\n",
        (unsigned long)(sent_sequence + 1U), (unsigned long)now,
        (int)sample.rx_yaw_raw, (int)sample.applied_yaw_raw,
        target, feedback, error, (int)sample.yaw_output,
        (unsigned int)sample.auto_active, (unsigned int)sample.remote_online,
        (unsigned long)sample.rx_count, (unsigned long)sample.applied_count,
        (int)sample.last_positive_raw, (int)sample.last_negative_raw,
        (unsigned long)sample.positive_count, (unsigned long)sample.negative_count,
        (unsigned int)sample.rx_bus);
    if (length > 0 && (unsigned int)length < sizeof(frame))
    {
        if (HAL_UART_Transmit_DMA(&huart6, (uint8_t *)frame, (uint16_t)length) == HAL_OK)
            ++sent_sequence;
    }
#elif VISION_TELEMETRY_FORMAT == VISION_TELEMETRY_JUSTFLOAT
    static struct
    {
        float channel[VISION_TELEMETRY_CHANNELS];
        uint8_t tail[4];
    } frame = {{0}, {0x00, 0x00, 0x80, 0x7f}};
    typedef char VofaFrameMustBe68Bytes[(sizeof(frame) == 68U) ? 1 : -1];
    (void)sizeof(VofaFrameMustBe68Bytes);
    frame.channel[0] = (float)sample.rx_yaw_raw;
    frame.channel[1] = (float)sample.applied_yaw_raw;
    frame.channel[2] = sample.yaw_target_rad;
    frame.channel[3] = sample.yaw_feedback_rad;
    frame.channel[4] = sample.yaw_error_rad;
    frame.channel[5] = (float)sample.yaw_output;
    frame.channel[6] = (float)sample.auto_active;
    frame.channel[7] = (float)sample.remote_online;
    frame.channel[8] = (float)sample.rx_count;
    frame.channel[9] = (float)sample.applied_count;
    frame.channel[10] = (float)sample.last_positive_raw;
    frame.channel[11] = (float)sample.last_negative_raw;
    frame.channel[12] = (float)sample.positive_count;
    frame.channel[13] = (float)sample.negative_count;
    frame.channel[14] = (float)sample.rx_bus;
    frame.channel[15] = (float)now * 0.001f;  // 板端时间，秒

    /* JustFloat：16 个小端 float32 + 00 00 80 7F；不混入任何 printf 文本。 */
    (void)HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&frame, (uint16_t)sizeof(frame));
#else
#error Unsupported VISION_TELEMETRY_FORMAT
#endif
#endif
}
