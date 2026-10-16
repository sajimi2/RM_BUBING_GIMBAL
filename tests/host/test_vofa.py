"""编译并运行真实 vofa_debug.c；用 HAL/RTOS 替身验证 DMA 缓冲生命周期与帧内容。"""
from pathlib import Path
import os
import shutil
import subprocess

root = Path(__file__).resolve().parents[2]
out = root / 'build/host-vofa'
out.mkdir(parents=True, exist_ok=True)
cc = os.environ.get('HOST_CC') or shutil.which('gcc')
if not cc:
    raise SystemExit('未找到宿主 GCC；设置 HOST_CC 为宿主 gcc.exe 路径。')

stub = r'''
#ifndef VOFA_STUB_H
#define VOFA_STUB_H
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
enum { HAL_UART_STATE_READY, HAL_UART_STATE_BUSY_TX, HAL_DMA_STATE_READY = 0,
       DMA_SxCR_EN = 1, DMA_IT_HT = 2, USART6_IRQn = 6, DMA2_Stream6_IRQn = 16,
       USART1_IRQn = 1, DMA2_Stream7_IRQn = 17, tskIDLE_PRIORITY = 0 };
typedef struct { uint32_t CR; } DMA_Stream_TypeDef;
typedef struct { int State; DMA_Stream_TypeDef *Instance; } DMA_HandleTypeDef;
typedef struct { int gState; DMA_HandleTypeDef *hdmatx; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1, huart6;
typedef struct { uint32_t CYCCNT; } DWT_Type;
extern DWT_Type test_dwt;
#define DWT (&test_dwt)
extern int critical_depth;
#define taskENTER_CRITICAL() (++critical_depth)
#define taskEXIT_CRITICAL() (--critical_depth)
#define pdMS_TO_TICKS(x) (x)
#define __HAL_DMA_DISABLE_IT(h, i) ((void)(h), (void)(i))
typedef uint32_t StackType_t;
typedef struct { uint32_t unused; } StaticTask_t;
void vTaskDelay(uint32_t delay);
void *xTaskCreateStatic(void (*fn)(void *), const char *, uint32_t, void *, uint32_t, StackType_t *, StaticTask_t *);
void HAL_NVIC_SetPriority(int, int, int);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, const uint8_t *, uint16_t);
typedef struct { float Yaw, Pitch; } INS_t;
typedef struct { uint16_t ecd; int16_t speed_rpm; } motor_measure_t;
typedef struct { int16_t yaw, pitch; } CAN_Send_to_MCU;
extern CAN_Send_to_MCU NX_data;
typedef struct {
    uint32_t vision_last_ms, yaw_last_ms, pitch_last_ms;
    uint8_t vision_seen, yaw_seen, pitch_seen;
} gimbal_rx_observation_t;
enum RobotState_e { RobotState_e_Powerless, RobotState_e_CommonCar };
const INS_t *get_INS(void);
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
const volatile gimbal_rx_observation_t *get_gimbal_rx_observation(void);
float get_yaw_target_angle(void);
float get_pitch_target_angle(void);
int robotIsAuto(void);
const enum RobotState_e *getRobotPresentMode(void);
#endif
'''
(out / 'stub.h').write_text(stub, encoding='utf-8')
for name in ('usart.h', 'FreeRTOS.h', 'task.h', 'CAN_recieve_send.h', 'NX_communicate.h',
             'INS_task.h', 'gimbal_task.h', 'robot_total_mode.h'):
    (out / name).write_text('#include "stub.h"\n', encoding='utf-8')

test = r'''
#include "stub.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "Debug/vofa_debug.c"
static DMA_Stream_TypeDef stream;
static DMA_HandleTypeDef dma = {0, &stream};
UART_HandleTypeDef huart6 = {0, &dma}, huart1 = {0, &dma};
DWT_Type test_dwt;
int critical_depth;
volatile uint8_t rc_online = 1;
CAN_Send_to_MCU NX_data = {16384, -8192};
static INS_t ins = {1.0f, -0.5f};
static motor_measure_t yaw = {4096, -100}, pitch = {2048, 200};
static gimbal_rx_observation_t rx = {990, 998, 999, 1, 1, 1};
static enum RobotState_e mode = RobotState_e_CommonCar;
static uint32_t now = 1000;
static int dma_calls, priorities;
static HAL_StatusTypeDef next_result = HAL_OK;
static const uint8_t *inflight;
uint32_t HAL_GetTick(void) { test_dwt.CYCCNT += 42; return now; }
const INS_t *get_INS(void) { return &ins; }
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void) { return &yaw; }
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void) { return &pitch; }
const volatile gimbal_rx_observation_t *get_gimbal_rx_observation(void) { return &rx; }
float get_yaw_target_angle(void) { return 1.5f; }
float get_pitch_target_angle(void) { return -0.25f; }
int robotIsAuto(void) { return 1; }
const enum RobotState_e *getRobotPresentMode(void) { return &mode; }
void vTaskDelay(uint32_t delay) { (void)delay; assert(!"sender must not wait"); }
void HAL_NVIC_SetPriority(int irq, int p, int sub) { (void)irq; assert(p == 7 && sub == 0); priorities++; }
void *xTaskCreateStatic(void (*fn)(void *), const char *name, uint32_t stack,
                       void *arg, uint32_t priority, StackType_t *buffer, StaticTask_t *tcb)
{
    (void)fn; (void)name; (void)arg;
    assert(stack == 256 && priority == 1 && buffer && tcb);
    return tcb;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t length)
{
    assert(critical_depth == 0 && length == 68);
    dma_calls++;
    if (next_result == HAL_OK) {
        inflight = data;
        uart->gState = HAL_UART_STATE_BUSY_TX;
        dma.State = 1;
        stream.CR = DMA_SxCR_EN;
    }
    return next_result;
}
static void ready(void) { huart6.gState = huart1.gState = 0; dma.State = 0; stream.CR = 0; }
static void near_value(float a, float b) { assert(fabsf(a - b) < 0.001f); }
int main(void)
{
    uint8_t saved[68];
    float values[16];
    VOFA_Debug_Start();
    assert(priorities == 2);
    vofa_send_once();
    assert(dma_calls == 1 && vofa_debug_stats.submitted == 1 && critical_depth == 0);
    memcpy(saved, inflight, 68);
    memcpy(values, inflight, 64);
    assert(memcmp(inflight + 64, "\x00\x00\x80\x7f", 4) == 0);
    /* 0～5 为 rad：视觉定点值还原，INS 和控制目标原样输出。 */
    near_value(values[0], 1.5708443f);
    near_value(values[1], -0.78542215f);
    near_value(values[2], 1.0f);
    near_value(values[3], -0.5f);
    near_value(values[4], 1.5f);
    near_value(values[5], -0.25f);
    near_value(values[6], 180); near_value(values[7], 90);
    near_value(values[8], -100); near_value(values[9], 200);
    near_value(values[10], 10); near_value(values[11], 2); near_value(values[12], 1);
    near_value(values[13], 1); near_value(values[14], 1); near_value(values[15], 1);
    NX_data.yaw = 0;
    vofa_send_once();
    assert(dma_calls == 1 && memcmp(inflight, saved, 68) == 0);
    huart6.gState = huart1.gState = 0; /* UART 已恢复，但 DMA 仍忙。 */
    vofa_send_once();
    assert(dma_calls == 1 && memcmp(inflight, saved, 68) == 0);
    dma.State = 0; /* 再检查硬件 EN，避免提前重用 DMA 缓冲。 */
    vofa_send_once();
    assert(dma_calls == 1 && vofa_debug_stats.skipped_busy == 3);
    ready(); next_result = HAL_BUSY; vofa_send_once();
    assert(vofa_debug_stats.skipped_busy == 4 && vofa_debug_stats.submitted == 1);
    next_result = HAL_ERROR; vofa_send_once();
    assert(vofa_debug_stats.start_errors == 1);
    next_result = HAL_OK; now = 3; rx.vision_last_ms = UINT32_MAX - 1;
    rx.pitch_seen = 0; mode = RobotState_e_Powerless; rc_online = 0;
    vofa_send_once(); memcpy(values, inflight, 64);
    near_value(values[0], 0); near_value(values[10], 5); near_value(values[12], -1);
    near_value(values[13], 0); near_value(values[14], 0);
    assert(vofa_debug_stats.submitted == 2 && vofa_debug_stats.snapshot_max_cycles == 42);
    puts("PASS: frame/units, DMA buffer lifetime, busy/error handling, tick wrap, freshness, low-priority static task");
}
'''
(out / 'test.c').write_text(test, encoding='utf-8')
exe = out / ('test_vofa.exe' if os.name == 'nt' else 'test_vofa')
subprocess.run([cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-I', str(out),
                '-I', str(root), str(out / 'test.c'), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
