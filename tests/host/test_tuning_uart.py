"""替换 HAL，验证真实串口驱动的 DMA 寿命、频率、超时和恢复。"""
from pathlib import Path
import os
import shutil
import subprocess

root = Path(__file__).resolve().parents[2]
out = root / 'build/host-tuning-uart'
out.mkdir(parents=True, exist_ok=True)
(out / 'usart.h').write_text(r'''
#include <stdint.h>
typedef struct { int gState; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart6;
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_UART_STATE_READY 0
#define __DMB() ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
uint32_t HAL_GetTick(void);
int HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, int);
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, int);
int HAL_UART_AbortReceive(UART_HandleTypeDef *);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t key) { (void)key; }
''', encoding='utf-8')
source = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "tuning_uart.c"
UART_HandleTypeDef huart6;
static uint32_t tick;
static uint8_t *receive_byte, *dma_bytes;
static int dma_length, transmit_fail, handled, aborted;
static uint16_t stream_period = 20;
uint32_t HAL_GetTick(void) { return tick; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, int length)
{ assert(uart == &huart6 && length == 1); receive_byte = byte; return HAL_OK; }
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *bytes, int length)
{
    assert(uart == &huart6);
    if (transmit_fail) return HAL_ERROR;
    assert(uart->gState == HAL_UART_STATE_READY);
    uart->gState = 1; dma_bytes = bytes; dma_length = length; return HAL_OK;
}
int HAL_UART_AbortReceive(UART_HandleTypeDef *uart) { (void)uart; ++aborted; return HAL_OK; }
void tuning_service_request(const tuning_message_t *req)
{ if (req->status == TUNING_OK) ++handled; }
uint16_t tuning_service_period_ms(void) { return stream_period; }
uint16_t tuning_service_frame(uint8_t bytes[TUNING_FRAME_SIZE])
{
    float values[TUNING_CHANNEL_COUNT] = {0};
    values[0] = (float)handled;
    return tuning_encode(values, bytes);
}
static void receive(const char *bytes)
{
    for (size_t i = 0; i < strlen(bytes); ++i) {
        *receive_byte = (uint8_t)bytes[i]; HAL_UART_RxCpltCallback(&huart6);
    }
}
static void complete(void) { huart6.gState = HAL_UART_STATE_READY; }
int main(void)
{
    uint8_t snapshot[TUNING_FRAME_SIZE];
    tuning_uart_init();
    assert(rx_byte == 0);
    tick = 19; tuning_uart_poll(); assert(huart6.gState == 0);
    tick = 20; tuning_uart_poll();
    assert(huart6.gState == 1 && dma_length == TUNING_FRAME_SIZE);
    complete();
    receive("gain:1000\n"); tuning_uart_poll(); assert(handled == 1 && huart6.gState == 1);
    memcpy(snapshot, dma_bytes, sizeof(snapshot));
    receive("ki:6000\n"); tuning_uart_poll();
    assert(handled == 1 && memcmp(snapshot, dma_bytes, sizeof(snapshot)) == 0);
    complete(); transmit_fail = 1; tuning_uart_poll();
    assert(handled == 2 && tuning_uart_tx_errors == 1 && tx_length != 0);
    transmit_fail = 0; tuning_uart_poll(); assert(huart6.gState == 1 && tx_length == 0);
    complete(); stream_period = 0;
    receive("gain:"); tuning_uart_poll(); assert(parser.count == 5);
    tick += 101; tuning_uart_poll(); assert(parser.count == 0 && parser.discard_line);
    receive("gain:5000\n"); tuning_uart_poll(); assert(handled == 2);
    complete(); receive("read:0\n"); tuning_uart_poll(); assert(handled == 3);
    complete();
    for (unsigned i = 0; i < RX_RING_SIZE + 10; ++i) receive("x");
    assert(tuning_uart_overflows > 0);
    tuning_uart_poll(); assert(rx_read == rx_write && !rx_overflow);
    receive("gain:5000\n"); tuning_uart_poll(); assert(handled == 3);
    complete(); receive("read:0\n"); tuning_uart_poll(); assert(handled == 4);
    complete(); receive("ki:60"); tuning_uart_poll();
    HAL_UART_ErrorCallback(&huart6); tuning_uart_poll();
    assert(aborted == 1 && tuning_uart_rx_errors == 1 && !rx_restart);
    receive("00\n"); tuning_uart_poll(); assert(handled == 4);
    complete(); receive("read:0\n"); tuning_uart_poll(); assert(handled == 5);
    complete(); receive("gain:1"); tuning_uart_poll();
    tick += 101;
    receive("000\n"); tuning_uart_poll(); assert(handled == 5);
    complete(); receive("read:0\n"); tuning_uart_poll(); assert(handled == 6);
    complete(); tick += 100; tuning_uart_poll(); assert(huart6.gState == 0);
    stream_period = 20;
    for (unsigned i = 0; i < 10; ++i) {
        tick += 20; tuning_uart_poll(); assert(huart6.gState == 1); complete();
    }
    puts("PASS: UART JustFloat streaming/20ms scheduling/DMA buffer/TX retry/timeout suffix rejection/overflow/error recovery");
}
'''
(out / 'test.c').write_text(source, encoding='utf-8')
cc = os.environ.get('HOST_CC') or shutil.which('gcc')
assert cc, 'Host GCC required'
exe = out / ('test_tuning_uart.exe' if os.name == 'nt' else 'test_tuning_uart')
subprocess.run([cc, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                '-I', str(out), '-I', str(root / 'Debug'), str(out / 'test.c'),
                str(root / 'Debug/tuning_protocol.c'), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
