#ifndef TUNING_PROTOCOL_H
#define TUNING_PROTOCOL_H

#include <stdint.h>
#include "tuning_config.h"

/* 上行：默认12通道，开启附加诊断后23通道；末尾均为JustFloat帧尾。 */
#if TUNING_DIAGNOSTICS_ENABLED && TUNING_VOFA_TX_ENABLED
#define TUNING_CHANNEL_COUNT 23U
#else
#define TUNING_CHANNEL_COUNT 12U
#endif
#define TUNING_FRAME_SIZE (4U * TUNING_CHANNEL_COUNT + 4U)
#define TUNING_LINE_SIZE 96U
#define TUNING_NAME_SIZE 24U
#define TUNING_VALUE_COUNT 3U

enum tuning_status
{
    TUNING_OK = 0,
    TUNING_BAD_COMMAND,
    TUNING_BAD_LENGTH,
    TUNING_BAD_PARAMETER,
    TUNING_OUT_OF_RANGE,
    TUNING_NOT_ENABLED,
    TUNING_WRONG_SOURCE
};

typedef struct
{
    char name[TUNING_NAME_SIZE];
    float values[TUNING_VALUE_COUNT];
    uint8_t count;
    uint8_t status;
} tuning_message_t;

typedef struct
{
    char line[TUNING_LINE_SIZE];
    uint16_t count;
    uint8_t discard_line;
    uint32_t rejected;
} tuning_parser_t;

void tuning_parser_reset(tuning_parser_t *parser);
/* 半行超时或 UART 出错后丢到下一个换行，不能把残余后缀当作新命令。 */
void tuning_parser_discard(tuning_parser_t *parser);
uint8_t tuning_parser_feed(tuning_parser_t *parser, uint8_t byte, tuning_message_t *message);
uint16_t tuning_encode(const float channels[TUNING_CHANNEL_COUNT], uint8_t *output);

#endif
