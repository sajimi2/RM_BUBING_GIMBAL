#ifndef VOFA_DEBUG_H
#define VOFA_DEBUG_H

#include <stdint.h>

/* C 板 3-pin 接口是 USART6（PG14/PG9）；4-pin 接口是 USART1（PA9/PB7）。 */
#define VOFA_UART_INDEX 6
#define VOFA_CHANNEL_COUNT 16
#define VOFA_PERIOD_MS 20

typedef struct
{
    uint32_t submitted;
    uint32_t skipped_busy;
    uint32_t start_errors;
    uint32_t snapshot_max_cycles;
} vofa_debug_stats_t;

/* 调试器可观察；submitted 表示已交给 DMA，不代表 PC 已成功收到。 */
extern volatile vofa_debug_stats_t vofa_debug_stats;
void VOFA_Debug_Start(void);

#endif
