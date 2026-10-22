#include "tuning_service.h"
#include "gimbal_task.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RAD_TO_DEG (180.0f / 3.1415926f)
#define DEG_TO_RAD (3.1415926f / 180.0f)

enum command_id
{
    CMD_UNKNOWN = 0,
    CMD_READ = 1,
    CMD_GAIN,
    CMD_OFFSET,
    CMD_KI,
    CMD_PARAMETERS,
    CMD_SOURCE,
    CMD_TARGET,
    CMD_STREAM,
    CMD_GRAVITY,
    CMD_MARK
};

typedef struct
{
    const char *name;
    uint8_t id;
    uint8_t count;
} tuning_command_t;

// 后续增加参数时，在表中注册名称、编号和参数个数。
static const tuning_command_t commands[] = {
    {"read", CMD_READ, 1},
    {"gain", CMD_GAIN, 1},
    {"offset", CMD_OFFSET, 1},
    {"ki", CMD_KI, 1},
    {"parameters", CMD_PARAMETERS, 3},
    {"source", CMD_SOURCE, 1},
    {"target", CMD_TARGET, 1},
    {"stream", CMD_STREAM, 1},
    {"gravity", CMD_GRAVITY, 1},
    {"mark", CMD_MARK, 1}
};

static uint16_t stream_period_ms = 20U;
static uint16_t command_count;
static uint8_t last_command;
static uint8_t last_status;
static uint16_t record_mark;
static uint8_t saved_gravity_valid;
static pitch_gravity_config_t saved_gravity;

static const tuning_command_t *find_command(const char *name)
{
    size_t i;
    for (i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i)
    {
        if (strcmp(name, commands[i].name) == 0)
            return &commands[i];
    }
    return NULL;
}

static uint8_t submit_pitch(float angle_rad)
{
    pitch_tuning_limits_t limits;
    uint8_t payload[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    int16_t raw;
    float decoded;

    gimbal_tuning_get_limits(&limits);
    if (!isfinite(angle_rad))
        return TUNING_OUT_OF_RANGE;
    if (angle_rad < limits.pitch_min_rad || angle_rad > limits.pitch_max_rad)
        return TUNING_OUT_OF_RANGE;
    // 继续复用视觉八字节载荷和 Pitch 符号，向零取整避免量化越界。
    raw = (int16_t)(-angle_rad * 32767.0f / 3.1415926f);
    decoded = -(float)raw * 3.1415926f / 32767.0f;
    if (decoded < limits.pitch_min_rad || decoded > limits.pitch_max_rad)
    {
        if (raw > 0)
            --raw;
        else if (raw < 0)
            ++raw;
    }
    payload[3] = (uint8_t)raw;
    payload[4] = (uint8_t)((uint16_t)raw >> 8);
    return gimbal_tuning_submit_vision(payload);
}

static uint8_t select_source(float value, const pitch_tuning_state_t *state)
{
    pitch_tuning_limits_t limits;
    float angle;
    uint8_t status;

    if (value == 0.0f)
        return gimbal_tuning_set_source(0);
    if (value != 1.0f)
        return TUNING_BAD_PARAMETER;
    // 已在串口模式时，重复点按钮不能把目标改回实测角度。
    if (state->source == 1U)
        return gimbal_tuning_set_source(1);
    angle = state->control.actual_angle_rad;
    gimbal_tuning_get_limits(&limits);
    if (!isfinite(angle))
        return TUNING_OUT_OF_RANGE;
    if (angle < limits.pitch_min_rad || angle > limits.pitch_max_rad)
        return TUNING_OUT_OF_RANGE;
    status = gimbal_tuning_set_source(1);
    if (status != TUNING_OK)
        return status;
    status = submit_pitch(angle);
    // 模式可能在两次调用之间变化；初始目标未接收就退回 CAN。
    if (status != TUNING_OK)
        gimbal_tuning_set_source(0);
    return status;
}

static uint8_t execute_command(uint8_t id, const float *values)
{
    pitch_tuning_state_t state;
    uint8_t status;
    float value = values[0];

    gimbal_tuning_get_state(&state);
    switch (id)
    {
    case CMD_READ:
        if (value != 0.0f)
            return TUNING_BAD_PARAMETER;
        return TUNING_OK;
    case CMD_GAIN:
        status = gimbal_tuning_set_gravity(value, state.gravity.offset_rad);
        break;
    case CMD_OFFSET:
        // 界面统一使用度，控制算法仍使用 rad。
        if (value < -180.0f || value > 180.0f)
            return TUNING_OUT_OF_RANGE;
        status = gimbal_tuning_set_gravity(state.gravity.gain, value * DEG_TO_RAD);
        break;
    case CMD_KI:
        return gimbal_tuning_set_parameters(state.gravity.gain, state.gravity.offset_rad, value);
    case CMD_PARAMETERS:
        if (values[1] < -180.0f || values[1] > 180.0f)
            return TUNING_OUT_OF_RANGE;
        status = gimbal_tuning_set_parameters(value, values[1] * DEG_TO_RAD, values[2]);
        break;
    case CMD_SOURCE:
        return select_source(value, &state);
    case CMD_TARGET:
        return submit_pitch(value * DEG_TO_RAD);
    case CMD_STREAM:
        // 115200 下 168 字节/帧，20 ms 周期约占 73% 带宽，禁止更快。
        if (value != truncf(value))
            return TUNING_BAD_PARAMETER;
        if (value != 0.0f && (value < 20.0f || value > 2000.0f))
            return TUNING_OUT_OF_RANGE;
        stream_period_ms = (uint16_t)value;
        return TUNING_OK;
    case CMD_GRAVITY:
        if (value == 0.0f)
        {
            status = gimbal_tuning_set_gravity(0.0f, state.gravity.offset_rad);
            // 连续关闭仍保留最初的值，不把零当作待恢复值。
            if (status == TUNING_OK && !saved_gravity_valid)
            {
                saved_gravity = state.gravity;
                saved_gravity_valid = 1;
            }
            return status;
        }
        if (value != 1.0f || !saved_gravity_valid)
            return TUNING_BAD_PARAMETER;
        status = gimbal_tuning_set_gravity(saved_gravity.gain, saved_gravity.offset_rad);
        break;
    case CMD_MARK:
        if (value != truncf(value) || value < 0.0f || value > 65535.0f)
            return TUNING_OUT_OF_RANGE;
        record_mark = (uint16_t)value;
        return TUNING_OK;
    default:
        return TUNING_BAD_COMMAND;
    }
    // 手动修改重力参数后，旧的“恢复关闭前参数”不再有效。
    if (status == TUNING_OK)
        saved_gravity_valid = 0;
    return status;
}

void tuning_service_request(const tuning_message_t *request)
{
    const tuning_command_t *command = find_command(request->name);
    ++command_count;
    last_command = CMD_UNKNOWN;
    last_status = request->status;
    if (command != NULL)
        last_command = command->id;
    // 格式错误的整行不执行，仍在下一帧报告错误码。
    if (last_status != TUNING_OK)
        return;
    if (command == NULL)
    {
        last_status = TUNING_BAD_COMMAND;
        return;
    }
    if (request->count != command->count)
    {
        last_status = TUNING_BAD_LENGTH;
        return;
    }
    last_status = execute_command(command->id, request->values);
}

uint16_t tuning_service_frame(uint8_t output[TUNING_FRAME_SIZE])
{
    pitch_tuning_state_t state;
    pitch_tuning_limits_t limits;
    const pitch_control_debug_t *control = &state.control;
    float channels[TUNING_CHANNEL_COUNT];
    uint16_t i;

    gimbal_tuning_get_state(&state);
    gimbal_tuning_get_limits(&limits);
    channels[0] = control->target_angle_rad * RAD_TO_DEG;
    channels[1] = control->actual_angle_rad * RAD_TO_DEG;
    channels[2] = control->angle_error_rad * RAD_TO_DEG;
    channels[3] = control->requested_angle_rad * RAD_TO_DEG;
    channels[4] = control->speed_rad_s;
    channels[5] = control->p_out;
    channels[6] = control->i_out;
    channels[7] = control->d_out;
    channels[8] = control->pid_unclamped_out;
    channels[9] = control->gravity_target_out;
    channels[10] = control->gravity_out;
    channels[11] = control->friction_out;
    channels[12] = control->total_unclamped_out;
    channels[13] = control->total_out;
    channels[14] = (float)control->motor_cmd;
    channels[15] = state.gravity.gain;
    channels[16] = state.gravity.offset_rad * RAD_TO_DEG;
    channels[17] = state.speed_ki;
    channels[18] = (float)state.source;
    channels[19] = (float)state.remote_online;
    channels[20] = (float)state.automatic;
    channels[21] = (float)control->control_enabled;
    channels[22] = (float)control->target_limited;
    channels[23] = (float)control->integral_limited;
    channels[24] = (float)control->pid_saturated;
    channels[25] = (float)control->total_saturated;
    channels[26] = (float)control->integral_blocked;
    channels[27] = (float)control->integral_separated;
    channels[28] = (float)control->gravity_limited;
    channels[29] = (float)control->gravity_config_invalid;
    channels[30] = (float)control->friction_still;
    channels[31] = (float)control->friction_active;
    channels[32] = (float)state.tick_ms * 0.001f;
    channels[33] = (float)last_command;
    channels[34] = (float)last_status;
    channels[35] = (float)command_count;
    channels[36] = (float)stream_period_ms;
    channels[37] = (float)record_mark;
    channels[38] = limits.pitch_min_rad * RAD_TO_DEG;
    channels[39] = limits.pitch_max_rad * RAD_TO_DEG;
    channels[40] = 1.0f;

    // Inf 会被误认作帧尾；非有限值改为零，并标记整帧无效。
    for (i = 0; i < TUNING_CHANNEL_COUNT - 1U; ++i)
    {
        if (!isfinite(channels[i]))
        {
            channels[i] = 0.0f;
            channels[40] = 0.0f;
        }
    }
    return tuning_encode(channels, output);
}

uint16_t tuning_service_period_ms(void)
{
    return stream_period_ms;
}
