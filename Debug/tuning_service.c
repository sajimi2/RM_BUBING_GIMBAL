#include "tuning_service.h"
#include "gimbal_task.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
    uint16_t id;
    const char *name;
    uint8_t unit; /* 0：控制量刻度，1：rad，2：速度环 Ki（每 2 ms） */
    size_t offset;
    float step;
} tuning_parameter_t;

/* 参数由表注册，读写与范围检查共用，不为每个参数增加一条串口命令。 */
static const tuning_parameter_t parameters[] = {
    {1, "pitch.gravity.gain", 0, offsetof(pitch_tuning_state_t, gravity) + offsetof(pitch_gravity_config_t, gain), 100.0f},
    {2, "pitch.gravity.offset", 1, offsetof(pitch_tuning_state_t, gravity) + offsetof(pitch_gravity_config_t, offset_rad), 0.01f},
    {3, "pitch.speed.ki", 2, offsetof(pitch_tuning_state_t, speed_ki), 100.0f}
};
#define PARAMETER_COUNT (sizeof(parameters) / sizeof(parameters[0]))

static uint16_t stream_period_ms;

static const tuning_parameter_t *find_parameter(uint16_t id)
{
    size_t i;
    for (i = 0; i < PARAMETER_COUNT; ++i)
        if (parameters[i].id == id)
            return &parameters[i];
    return NULL;
}

static float parameter_value(const pitch_tuning_state_t *state, const tuning_parameter_t *parameter)
{
    float value;
    memcpy(&value, (const uint8_t *)state + parameter->offset, sizeof(value));
    return value;
}

static float parameter_max(const pitch_tuning_limits_t *limits, const tuning_parameter_t *parameter)
{
    if (parameter->unit == 2U)
        return limits->speed_ki_max;
    return parameter->unit == 0U ? limits->gain_max : limits->offset_max_rad;
}

static float parameter_min(float max_value, const tuning_parameter_t *parameter)
{
    return parameter->unit == 2U ? 0.0f : -max_value;
}

static uint8_t set_parameters(const tuning_message_t *request, tuning_message_t *reply)
{
    pitch_tuning_state_t state;
    pitch_tuning_limits_t limits;
    uint8_t count, i, j, status;
    uint16_t id;
    float value;
    const tuning_parameter_t *parameter;

    if (request->length < 1U)
        return TUNING_BAD_LENGTH;
    count = request->payload[0];
    if (count == 0U || count > PARAMETER_COUNT || request->length != 1U + 6U * count)
        return TUNING_BAD_LENGTH;
    gimbal_tuning_get_state(&state);
    gimbal_tuning_get_limits(&limits);
    for (i = 0; i < count; ++i)
    {
        id = tuning_read_u16(request->payload + 1U + 6U * i);
        parameter = find_parameter(id);
        if (parameter == NULL)
            return TUNING_BAD_PARAMETER;
        for (j = 0; j < i; ++j)
            if (id == tuning_read_u16(request->payload + 1U + 6U * j))
                return TUNING_BAD_PARAMETER;
        value = tuning_read_float(request->payload + 3U + 6U * i);
        const float maximum = parameter_max(&limits, parameter);
        if (!isfinite(value) || value < parameter_min(maximum, parameter) || value > maximum)
            return TUNING_OUT_OF_RANGE;
        memcpy((uint8_t *)&state + parameter->offset, &value, sizeof(value));
    }
    /* 全部检查通过后一起提交。任意一项错误都不修改运行参数。 */
    status = gimbal_tuning_set_parameters(state.gravity.gain, state.gravity.offset_rad, state.speed_ki);
    if (status == TUNING_OK)
    {
        memcpy(reply->payload + 1, request->payload, request->length);
        reply->length = request->length + 1U;
    }
    return status;
}

void tuning_service_state(tuning_message_t *message)
{
    pitch_tuning_state_t state;
    const pitch_control_debug_t *c = &state.control;
    float values[15];
    uint16_t flags;
    uint8_t i;
    gimbal_tuning_get_state(&state);
    flags = (uint16_t)((c->control_enabled != 0U) |
        ((c->target_limited != 0U) << 1) | ((c->integral_limited != 0U) << 2) |
        ((c->pid_saturated != 0U) << 3) | ((c->total_saturated != 0U) << 4) |
        ((c->integral_blocked != 0U) << 5) | ((c->integral_separated != 0U) << 6) |
        ((c->gravity_limited != 0U) << 7) | ((c->gravity_config_invalid != 0U) << 8));
    message->payload[0] = TUNING_OK;
    tuning_write_u32(message->payload + 1, state.tick_ms);
    message->payload[5] = state.source;
    message->payload[6] = state.remote_online;
    message->payload[7] = state.automatic;
    tuning_write_u16(message->payload + 8, flags);
    values[0] = state.gravity.gain;
    values[1] = state.gravity.offset_rad;
    values[2] = c->requested_angle_rad;
    values[3] = c->target_angle_rad;
    values[4] = c->actual_angle_rad;
    values[5] = c->speed_rad_s;
    values[6] = c->angle_error_rad;
    values[7] = c->p_out;
    values[8] = c->i_out;
    values[9] = c->d_out;
    values[10] = c->pid_unclamped_out;
    values[11] = c->gravity_target_out;
    values[12] = c->gravity_out;
    values[13] = c->total_unclamped_out;
    values[14] = c->total_out;
    for (i = 0; i < 15U; ++i)
        tuning_write_float(message->payload + 10U + 4U * i, values[i]);
    tuning_write_u16(message->payload + 70, (uint16_t)c->motor_cmd);
    tuning_write_float(message->payload + 72, state.speed_ki);
    message->length = 76;
}

void tuning_service_request(const tuning_message_t *request, tuning_message_t *reply)
{
    pitch_tuning_state_t state;
    pitch_tuning_limits_t limits;
    const tuning_parameter_t *parameter;
    uint16_t index, period;
    float max_value;
    uint8_t status = TUNING_OK;

    reply->command = request->command | 0x80U;
    reply->sequence = request->sequence;
    reply->length = 1;
    switch (request->command)
    {
    case TUNING_HELLO:
        if (request->length != 0U) { status = TUNING_BAD_LENGTH; break; }
        gimbal_tuning_get_limits(&limits);
        reply->payload[1] = TUNING_VERSION;
        tuning_write_u16(reply->payload + 2, (uint16_t)PARAMETER_COUNT);
        tuning_write_float(reply->payload + 4, limits.pitch_min_rad);
        tuning_write_float(reply->payload + 8, limits.pitch_max_rad);
        tuning_write_float(reply->payload + 12, limits.gain_max);
        tuning_write_float(reply->payload + 16, limits.offset_max_rad);
        tuning_write_float(reply->payload + 20, limits.gravity_max_out);
        tuning_write_float(reply->payload + 24, limits.total_max_out);
        memcpy(reply->payload + 28, "gimbal-pitch", 13);
        reply->length = 41;
        break;
    case TUNING_PARAM_INFO:
        if (request->length != 2U) { status = TUNING_BAD_LENGTH; break; }
        index = tuning_read_u16(request->payload);
        if (index >= PARAMETER_COUNT) { status = TUNING_BAD_PARAMETER; break; }
        parameter = &parameters[index];
        gimbal_tuning_get_limits(&limits);
        gimbal_tuning_get_state(&state);
        max_value = parameter_max(&limits, parameter);
        tuning_write_u16(reply->payload + 1, parameter->id);
        reply->payload[3] = parameter->unit;
        tuning_write_float(reply->payload + 4, parameter_min(max_value, parameter));
        tuning_write_float(reply->payload + 8, max_value);
        tuning_write_float(reply->payload + 12, parameter->step);
        tuning_write_float(reply->payload + 16, parameter_value(&state, parameter));
        memcpy(reply->payload + 20, parameter->name, strlen(parameter->name) + 1U);
        reply->length = (uint16_t)(21U + strlen(parameter->name));
        break;
    case TUNING_PARAM_GET:
        if (request->length != 2U) { status = TUNING_BAD_LENGTH; break; }
        parameter = find_parameter(tuning_read_u16(request->payload));
        if (parameter == NULL) { status = TUNING_BAD_PARAMETER; break; }
        gimbal_tuning_get_state(&state);
        tuning_write_u16(reply->payload + 1, parameter->id);
        tuning_write_float(reply->payload + 3, parameter_value(&state, parameter));
        reply->length = 7;
        break;
    case TUNING_PARAM_SET:
        status = set_parameters(request, reply);
        break;
    case TUNING_STATE:
        if (request->length != 0U) { status = TUNING_BAD_LENGTH; break; }
        tuning_service_state(reply);
        break;
    case TUNING_STREAM:
        if (request->length != 2U) { status = TUNING_BAD_LENGTH; break; }
        period = tuning_read_u16(request->payload);
        if (period != 0U && (period < 100U || period > 2000U))
        { status = TUNING_OUT_OF_RANGE; break; }
        stream_period_ms = period;
        tuning_write_u16(reply->payload + 1, period);
        reply->length = 3;
        break;
    case TUNING_SOURCE:
        if (request->length != 1U) { status = TUNING_BAD_LENGTH; break; }
        status = gimbal_tuning_set_source(request->payload[0]);
        if (status == TUNING_OK) { reply->payload[1] = request->payload[0]; reply->length = 2; }
        break;
    case TUNING_VISION:
        if (request->length != 8U) { status = TUNING_BAD_LENGTH; break; }
        status = gimbal_tuning_submit_vision(request->payload);
        break;
    default:
        status = TUNING_BAD_COMMAND;
        break;
    }
    reply->payload[0] = status;
}

uint16_t tuning_service_period_ms(void)
{
    return stream_period_ms;
}
