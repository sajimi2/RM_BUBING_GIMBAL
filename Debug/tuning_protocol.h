#ifndef TUNING_PROTOCOL_H
#define TUNING_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define TUNING_VERSION 1U
#define TUNING_MAX_PAYLOAD 128U
#define TUNING_FRAME_SIZE (TUNING_MAX_PAYLOAD + 10U)

enum tuning_command
{
    TUNING_HELLO = 0x01,
    TUNING_PARAM_INFO = 0x02,
    TUNING_PARAM_GET = 0x10,
    TUNING_PARAM_SET = 0x11,
    TUNING_STATE = 0x20,
    TUNING_STREAM = 0x21,
    TUNING_SOURCE = 0x30,
    TUNING_VISION = 0x31,
    TUNING_TELEMETRY = 0x40
};

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
    uint8_t command;
    uint16_t sequence;
    uint16_t length;
    uint8_t payload[TUNING_MAX_PAYLOAD];
} tuning_message_t;

typedef struct
{
    uint8_t bytes[TUNING_FRAME_SIZE];
    uint16_t count;
    uint32_t rejected;
} tuning_parser_t;

/* 纯协议层，不依赖串口、RTOS 或电机；所有多字节数据使用小端序。 */
void tuning_parser_reset(tuning_parser_t *parser);
uint8_t tuning_parser_feed(tuning_parser_t *parser, uint8_t byte, tuning_message_t *message);
uint16_t tuning_encode(const tuning_message_t *message, uint8_t *output);
uint16_t tuning_crc16(const uint8_t *data, uint16_t length);
uint16_t tuning_read_u16(const uint8_t *data);
void tuning_write_u16(uint8_t *data, uint16_t value);
void tuning_write_u32(uint8_t *data, uint32_t value);
float tuning_read_float(const uint8_t *data);
void tuning_write_float(uint8_t *data, float value);

#endif
