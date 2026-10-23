#include "tuning_protocol.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

void tuning_parser_reset(tuning_parser_t *parser)
{
    parser->count = 0;
    parser->discard_line = 0;
}

void tuning_parser_discard(tuning_parser_t *parser)
{
    parser->count = 0;
    parser->discard_line = 1;
    ++parser->rejected;
}

static uint8_t is_space(char value)
{
    return value == ' ' || value == '\t';
}

static uint8_t parse_line(char *line, tuning_message_t *message)
{
    char *cursor = line;
    char *start;
    char *end;
    uint16_t length;
    float value;

    memset(message, 0, sizeof(*message));
    message->status = TUNING_BAD_LENGTH;
    while (is_space(*cursor))
        ++cursor;
    start = cursor;
    while (*cursor != '\0')
    {
        // 多词命令用下划线分隔，仍不接受空格或其他符号混入名称。
        if (*cursor == '_')
        {
            ++cursor;
            continue;
        }
        if (*cursor < 'a' || *cursor > 'z')
            break;
        ++cursor;
    }
    length = (uint16_t)(cursor - start);
    // 名称仅用小写字母和下划线；未知名称交由服务层报告。
    if (length == 0U || length >= TUNING_NAME_SIZE)
        return 1;
    memcpy(message->name, start, length);
    while (is_space(*cursor))
        ++cursor;
    if (*cursor != ':')
        return 1;
    ++cursor;
    while (1)
    {
        while (is_space(*cursor))
            ++cursor;
        if (message->count == TUNING_VALUE_COUNT)
            return 1;
        start = cursor;
        // 不接受 nan、inf 和十六进制浮点数。
        while (*cursor != '\0')
        {
            if (strchr("0123456789+-.eE", *cursor) == NULL)
                break;
            ++cursor;
        }
        if (cursor == start)
            return 1;
        value = strtof(start, &end);
        if (end != cursor)
            return 1;
        if (!isfinite(value))
        {
            message->status = TUNING_OUT_OF_RANGE;
            return 1;
        }
        message->values[message->count] = value;
        ++message->count;
        while (is_space(*cursor))
            ++cursor;
        // 数值后只能结束，或者用逗号继续下一项。
        if (*cursor == '\0')
        {
            message->status = TUNING_OK;
            return 1;
        }
        if (*cursor != ',')
            return 1;
        ++cursor;
    }
}

uint8_t tuning_parser_feed(tuning_parser_t *parser, uint8_t byte, tuning_message_t *message)
{
    // 接受 LF、CR、CRLF；CRLF 后面的空行不生成第二条命令。
    if (byte == '\n' || byte == '\r')
    {
        if (parser->discard_line)
        {
            tuning_parser_reset(parser);
            memset(message, 0, sizeof(*message));
            message->status = TUNING_BAD_LENGTH;
            return 1;
        }
        if (parser->count == 0U)
            return 0;
        parser->line[parser->count] = '\0';
        parser->count = 0;
        return parse_line(parser->line, message);
    }
    if (parser->discard_line)
        return 0;
    // 旧二进制帧、嵌入 NUL 或超长命令整行丢弃，不执行其中的后缀。
    if (byte != '\t' && (byte < 0x20U || byte > 0x7eU))
    {
        tuning_parser_discard(parser);
        return 0;
    }
    if (parser->count >= TUNING_LINE_SIZE - 1U)
    {
        tuning_parser_discard(parser);
        return 0;
    }
    parser->line[parser->count] = (char)byte;
    ++parser->count;
    return 0;
}

uint16_t tuning_encode(const float channels[TUNING_CHANNEL_COUNT], uint8_t *output)
{
    uint16_t i;
    uint32_t bits;
    for (i = 0; i < TUNING_CHANNEL_COUNT; ++i)
    {
        memcpy(&bits, &channels[i], sizeof(bits));
        output[4U * i] = (uint8_t)bits;
        output[4U * i + 1U] = (uint8_t)(bits >> 8);
        output[4U * i + 2U] = (uint8_t)(bits >> 16);
        output[4U * i + 3U] = (uint8_t)(bits >> 24);
    }
    output[4U * TUNING_CHANNEL_COUNT] = 0x00;
    output[4U * TUNING_CHANNEL_COUNT + 1U] = 0x00;
    output[4U * TUNING_CHANNEL_COUNT + 2U] = 0x80;
    output[4U * TUNING_CHANNEL_COUNT + 3U] = 0x7f;
    return TUNING_FRAME_SIZE;
}
