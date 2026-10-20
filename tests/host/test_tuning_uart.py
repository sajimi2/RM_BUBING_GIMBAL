"""给实际 UART 驱动替换 HAL，验证 DMA 缓冲寿命、溢出恢复和半包超时。"""
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
static uint8_t *receive_byte;
static uint8_t *dma_bytes;
static int dma_length, transmit_fail, handled, aborted;
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
void tuning_service_request(const tuning_message_t *req, tuning_message_t *res)
{ ++handled; *res = *req; res->command |= 128; }
uint16_t tuning_service_period_ms(void) { return 0; }
void tuning_service_state(tuning_message_t *message) { (void)message; }
static void receive(const uint8_t *bytes, unsigned length)
{
    for (unsigned i = 0; i < length; ++i) {
        *receive_byte = bytes[i]; HAL_UART_RxCpltCallback(&huart6);
    }
}
int main(void)
{
    tuning_message_t req = {.command = TUNING_PARAM_SET, .sequence = 1, .length = 7};
    uint8_t frame[TUNING_FRAME_SIZE], snapshot[TUNING_FRAME_SIZE];
    unsigned size = tuning_encode(&req, frame);
    tuning_uart_init();
    receive(frame, size); tuning_uart_poll(); assert(handled == 1);
    tuning_uart_poll(); assert(huart6.gState == 1 && dma_length == (int)size);
    memcpy(snapshot, dma_bytes, size);
    receive(frame, size); tuning_uart_poll();
    assert(handled == 1 && memcmp(snapshot, dma_bytes, size) == 0);
    huart6.gState = HAL_UART_STATE_READY;
    tuning_uart_poll(); assert(handled == 2);
    transmit_fail = 1; tuning_uart_poll(); assert(tuning_uart_tx_errors == 1);
    assert(tx_length != 0 && huart6.gState == HAL_UART_STATE_READY);
    transmit_fail = 0; tuning_uart_poll(); assert(huart6.gState == 1);
    huart6.gState = HAL_UART_STATE_READY;
    receive(frame, 5); tuning_uart_poll(); assert(parser.count == 5);
    tick += 101; tuning_uart_poll(); assert(parser.count == 0);
    receive(frame + 5, size - 5); tuning_uart_poll(); assert(handled == 2);
    receive(frame, size); tuning_uart_poll(); assert(handled == 3);
    tuning_uart_poll(); huart6.gState = HAL_UART_STATE_READY;
    for (unsigned i = 0; i < RX_RING_SIZE + 10; ++i) receive(frame, 1);
    assert(tuning_uart_overflows > 0);
    tuning_uart_poll(); assert(rx_read == rx_write && !rx_overflow);
    receive(frame, size); tuning_uart_poll(); assert(handled == 4);
    tuning_uart_poll(); huart6.gState = HAL_UART_STATE_READY;
    receive(frame, 5); tuning_uart_poll();
    HAL_UART_ErrorCallback(&huart6); tuning_uart_poll();
    assert(aborted == 1 && tuning_uart_rx_errors == 1 && !rx_restart && !parser.count);
    receive(frame, size); tuning_uart_poll(); assert(handled == 5);
    puts("PASS: actual UART transport; fragmented input/timeout/DMA buffer lifetime/TX retry/overflow/UART error recovery");
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
