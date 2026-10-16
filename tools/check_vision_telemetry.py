"""主机验证真实遥测实现的线格式、发送节拍及 DMA 缓冲区寿命；不连接板子。"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

UART_STUB = r'''
#ifndef USART_H
#define USART_H
#include <stdint.h>
typedef struct { uint32_t gState; } UART_HandleTypeDef;
enum { HAL_UART_STATE_READY, HAL_UART_STATE_BUSY_TX };
#define HAL_OK 0
extern UART_HandleTypeDef huart6;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
'''

CHECK = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "NX_communicate.h"
#include "vision_telemetry.h"
UART_HandleTypeDef huart6;
volatile VisionDebug_t vision_debug;
static uint32_t tick, mask, sends;
static uint8_t *dma_bytes;
static uint16_t dma_length;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1; }
void __set_PRIMASK(uint32_t value) { mask = value; }
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *bytes, uint16_t size) {
    assert(uart == &huart6 && uart->gState == HAL_UART_STATE_READY);
    dma_bytes = bytes; dma_length = size; ++sends;
    uart->gState = HAL_UART_STATE_BUSY_TX;
    return 0;
}
static float channel(unsigned int n) {
    float value; memcpy(&value, dma_bytes + n * 4, 4); return value;
}
int main(void) {
    const uint8_t tail[] = {0, 0, 0x80, 0x7f};
    const uint8_t negative_three[] = {0, 0, 0x40, 0xc0};
    vision_debug.rx_yaw_raw = -31290;
    vision_debug.applied_yaw_raw = -31290;
    vision_debug.yaw_target_rad = -3.0f;
    vision_debug.yaw_feedback_rad = 0.25f;
    vision_debug.yaw_error_rad = 3.0331853f;
    vision_debug.yaw_output = -30000;
    vision_debug.auto_active = vision_debug.remote_online = 1;
    vision_debug.rx_count = 30; vision_debug.applied_count = 20;
    vision_debug.last_positive_raw = 31290; vision_debug.last_negative_raw = -31290;
    vision_debug.positive_count = 10; vision_debug.negative_count = 20;
    vision_debug.rx_bus = 2;
    tick = 99; VisionTelemetry_Poll(); assert(sends == 0);
    tick = 100; VisionTelemetry_Poll();
    assert(sends == 1 && dma_length == 68 && mask == 0);
    assert(memcmp(dma_bytes + 64, tail, 4) == 0);
    assert(memcmp(dma_bytes + 8, negative_three, 4) == 0);
    assert(channel(0) == -31290 && channel(1) == -31290);
    assert(channel(3) == 0.25f && fabsf(channel(4) - 3.0331853f) < 0.00001f);
    assert(channel(5) == -30000 && channel(6) == 1 && channel(7) == 1);
    assert(channel(8) == 30 && channel(9) == 20);
    assert(channel(10) == 31290 && channel(11) == -31290);
    assert(channel(12) == 10 && channel(13) == 20 && channel(14) == 2);
    assert(fabsf(channel(15) - 0.1f) < 0.00001f);
    puts("PASS: 68-byte JustFloat frame, little-endian signs and channel units");

    uint8_t in_flight[68]; memcpy(in_flight, dma_bytes, 68);
    vision_debug.rx_yaw_raw = 31290; vision_debug.yaw_target_rad = 3.0f;
    tick = 200; VisionTelemetry_Poll();
    assert(sends == 1 && memcmp(in_flight, dma_bytes, 68) == 0);
    huart6.gState = HAL_UART_STATE_READY;
    tick = 299; VisionTelemetry_Poll(); assert(sends == 1);
    mask = 1; tick = 300; VisionTelemetry_Poll();
    assert(sends == 2 && channel(0) == 31290 && channel(2) == 3.0f && mask == 1);
    puts("PASS: DMA busy buffer stays unchanged, next sample and IRQ mask preserved");

    huart6.gState = HAL_UART_STATE_READY;
    tick = UINT32_MAX - 49U; VisionTelemetry_Poll(); assert(sends == 3);
    huart6.gState = HAL_UART_STATE_READY;
    tick = 49; VisionTelemetry_Poll(); assert(sends == 3);
    tick = 50; VisionTelemetry_Poll(); assert(sends == 4);
    puts("PASS: 100 ms pacing across HAL tick wraparound");
    puts("Offline checks passed; UART/Bluetooth/VOFA still need hardware validation.");
}
'''

TEXT_CHECK = CHECK[:CHECK.index("static float channel")] + r'''
int main(void) {
    vision_debug.rx_yaw_raw = -31290; vision_debug.applied_yaw_raw = -31290;
    vision_debug.yaw_target_rad = -2.999983f;
    vision_debug.yaw_feedback_rad = 0.25f; vision_debug.yaw_error_rad = -0.125f;
    vision_debug.yaw_output = -30000;
    vision_debug.auto_active = vision_debug.remote_online = 1;
    vision_debug.last_positive_raw = 31290; vision_debug.last_negative_raw = -31290;
    tick = 499; VisionTelemetry_Poll(); assert(sends == 0);
    tick = 500; VisionTelemetry_Poll(); assert(sends == 1 && mask == 0);
    assert(strstr((char *)dma_bytes, "seq=1 ms=500 raw=-31290 used=-31290") != NULL);
    assert(strstr((char *)dma_bytes, "target=-3.000 fb=+0.250 err=-0.125 out=-30000 auto=1 rc=1") != NULL);
    assert(strstr((char *)dma_bytes, "pos=31290 neg=-31290") != NULL);
    assert(strlen((char *)dma_bytes) == dma_length);
    assert(dma_bytes[dma_length-2] == '\r' && dma_bytes[dma_length-1] == '\n');
    char in_flight[256]; memcpy(in_flight, dma_bytes, dma_length);
    vision_debug.yaw_target_rad = 2.999983f;
    tick = 1000; VisionTelemetry_Poll();
    assert(sends == 1 && memcmp(in_flight, dma_bytes, dma_length) == 0);
    huart6.gState = HAL_UART_STATE_READY;
    tick = 1499; VisionTelemetry_Poll(); assert(sends == 1);
    mask = 1; tick = 1500; VisionTelemetry_Poll();
    assert(sends == 2 && mask == 1);
    assert(strstr((char *)dma_bytes, "seq=2 ms=1500") != NULL);
    assert(strstr((char *)dma_bytes, "target=+3.000") != NULL);
    puts("PASS: signed readable rad, counters, CRLF, 500 ms pacing and DMA lifetime");

    huart6.gState = HAL_UART_STATE_READY;
    vision_debug.rx_count = vision_debug.applied_count = UINT32_MAX;
    vision_debug.positive_count = vision_debug.negative_count = UINT32_MAX;
    vision_debug.rx_yaw_raw = vision_debug.applied_yaw_raw = INT16_MIN;
    vision_debug.yaw_target_rad = -10000.0f;
    vision_debug.yaw_feedback_rad = INFINITY; vision_debug.yaw_error_rad = NAN;
    tick = UINT32_MAX - 249U; VisionTelemetry_Poll(); assert(sends == 3);
    assert(strstr((char *)dma_bytes, "target=-10000.000 fb=invalid err=invalid") != NULL);
    assert(strstr((char *)dma_bytes, "rx=4294967295 apply=4294967295") != NULL);
    assert(dma_length < 256 && dma_length * 2 * 10 < 9600);
    huart6.gState = HAL_UART_STATE_READY;
    tick = 249; VisionTelemetry_Poll(); assert(sends == 3);
    tick = 250; VisionTelemetry_Poll(); assert(sends == 4);
    puts("PASS: long counters, invalid-angle handling, UART capacity and tick wraparound");
}
'''

DISABLED_CHECK = CHECK[:CHECK.index("static float channel")] + r'''
int main(void) {
    tick = UINT32_MAX; mask = 1; VisionTelemetry_Poll();
    assert(sends == 0 && mask == 1);
    puts("PASS: disabling telemetry prevents UART transmission");
}
'''

with tempfile.TemporaryDirectory(prefix="bubingimbal-telemetry-check-") as directory:
    work = Path(directory)
    (work / "usart.h").write_text(UART_STUB, encoding="utf-8")
    for name, source, flags in (
        ("justfloat", CHECK, ["-DVISION_TELEMETRY_FORMAT=2", "-DVISION_TELEMETRY_ENABLED=1"]),
        ("text", TEXT_CHECK, ["-DVISION_TELEMETRY_FORMAT=1", "-DVISION_TELEMETRY_ENABLED=1"]),
        ("disabled", DISABLED_CHECK, ["-DVISION_TELEMETRY_ENABLED=0"]),
    ):
        (work / "check.c").write_text(source, encoding="utf-8")
        subprocess.run([
            "gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", *flags,
            "-I", str(work), "-I", str(ROOT / "Applications"),
            "-I", str(ROOT / "Task"), "-I", str(ROOT / "Debug"),
            str(work / "check.c"), str(ROOT / "Debug/vision_telemetry.c"),
            "-o", str(work / (name + ".exe")),
        ], check=True)
        subprocess.run([str(work / (name + ".exe"))], check=True)
