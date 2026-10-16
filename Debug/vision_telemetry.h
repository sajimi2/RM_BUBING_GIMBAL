#ifndef VISION_TELEMETRY_H
#define VISION_TELEMETRY_H

/* 外壳 UART1 三针接口 = MCU USART6：TX PG14、RX PG9，另有 GND，无电源脚。
 * 与 C 板相接的 BT24 串口须匹配 USART6；当前 USART6/IOC 为 9600、8N1。
 */
/* 当前使用安卓蓝牙助手读文本；宏可由离线测试覆盖。 */
#ifndef VISION_TELEMETRY_ENABLED
#define VISION_TELEMETRY_ENABLED 1
#endif
#define VISION_TELEMETRY_TEXT 1
#define VISION_TELEMETRY_JUSTFLOAT 2
#ifndef VISION_TELEMETRY_FORMAT
#define VISION_TELEMETRY_FORMAT VISION_TELEMETRY_TEXT
#endif
#if VISION_TELEMETRY_FORMAT == VISION_TELEMETRY_TEXT
/* 文本含诊断字段名，每秒两行，留出 9600 baud 链路余量。 */
#define VISION_TELEMETRY_PERIOD_MS 500U
#else
#define VISION_TELEMETRY_PERIOD_MS 100U
#endif
#define VISION_TELEMETRY_CHANNELS 16U

/* 仅由原 test_task 调用，DMA 忙时跳过本帧，不等待、不干预控制。 */
void VisionTelemetry_Poll(void);

#endif
