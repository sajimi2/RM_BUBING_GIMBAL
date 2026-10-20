#include "tuning_protocol.h"
#include <string.h>

uint16_t tuning_read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

void tuning_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

void tuning_write_u32(uint8_t *data, uint32_t value)
{
    tuning_write_u16(data, (uint16_t)value);
    tuning_write_u16(data + 2, (uint16_t)(value >> 16));
}

float tuning_read_float(const uint8_t *data)
{
    uint32_t bits = (uint32_t)tuning_read_u16(data) |
                    ((uint32_t)tuning_read_u16(data + 2) << 16);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void tuning_write_float(uint8_t *data, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    tuning_write_u32(data, bits);
}

uint16_t tuning_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xffffU;
    uint16_t i;
    uint8_t bit;
    for (i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
    }
    return crc;
}

void tuning_parser_reset(tuning_parser_t *parser)
{
    parser->count = 0;
}

/* 出错后保留缓冲区中下一处帧头，避免把紧随坏包的好包一起丢掉。 */
static void parser_resync(tuning_parser_t *parser)
{
    uint16_t i;
    for (i = 1; i < parser->count; ++i)
    {
        if (parser->bytes[i] == 0xa5U &&
            (i + 1U == parser->count || parser->bytes[i + 1U] == 0x5aU))
        {
            parser->count -= i;
            memmove(parser->bytes, parser->bytes + i, parser->count);
            return;
        }
    }
    parser->count = 0;
}

uint8_t tuning_parser_feed(tuning_parser_t *parser, uint8_t byte, tuning_message_t *message)
{
    uint16_t length, frame_length;
    if (parser->count == sizeof(parser->bytes))
        parser_resync(parser);
    parser->bytes[parser->count++] = byte;

    while (parser->count != 0U)
    {
        if (parser->bytes[0] != 0xa5U ||
            (parser->count >= 2U && parser->bytes[1] != 0x5aU))
        {
            parser_resync(parser);
            continue;
        }
        if (parser->count < 8U)
            return 0;
        length = tuning_read_u16(parser->bytes + 6);
        if (parser->bytes[2] != TUNING_VERSION || length > TUNING_MAX_PAYLOAD)
        {
            ++parser->rejected;
            parser_resync(parser);
            continue;
        }
        frame_length = length + 10U;
        if (parser->count < frame_length)
            return 0;
        if (tuning_crc16(parser->bytes + 2, length + 6U) !=
            tuning_read_u16(parser->bytes + length + 8U))
        {
            ++parser->rejected;
            parser_resync(parser);
            continue;
        }
        message->command = parser->bytes[3];
        message->sequence = tuning_read_u16(parser->bytes + 4);
        message->length = length;
        memcpy(message->payload, parser->bytes + 8, length);
        parser->count -= frame_length;
        memmove(parser->bytes, parser->bytes + frame_length, parser->count);
        return 1;
    }
    return 0;
}

uint16_t tuning_encode(const tuning_message_t *message, uint8_t *output)
{
    if (message->length > TUNING_MAX_PAYLOAD)
        return 0;
    output[0] = 0xa5;
    output[1] = 0x5a;
    output[2] = TUNING_VERSION;
    output[3] = message->command;
    tuning_write_u16(output + 4, message->sequence);
    tuning_write_u16(output + 6, message->length);
    memcpy(output + 8, message->payload, message->length);
    tuning_write_u16(output + 8 + message->length,
                     tuning_crc16(output + 2, message->length + 6U));
    return message->length + 10U;
}
