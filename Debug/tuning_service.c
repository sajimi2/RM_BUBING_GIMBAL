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
    CMD_MARK,
    CMD_YAW,
    CMD_YAW_WAVE,
    CMD_YAW_PING,
    CMD_YAW_STOP
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
    {"mark", CMD_MARK, 1},
    {"yaw", CMD_YAW, 1},
    {"yaw_wave", CMD_YAW_WAVE, 3},
    {"yaw_ping", CMD_YAW_PING, 1},
    {"yaw_stop", CMD_YAW_STOP, 1}
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

static uint8_t submit_angles(float pitch_rad, float yaw_rad)
{
    pitch_tuning_limits_t limits;
    uint8_t payload[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    int16_t raw;
    float decoded;

    gimbal_tuning_get_limits(&limits);
    if (!isfinite(pitch_rad) || !isfinite(yaw_rad))
        return TUNING_OUT_OF_RANGE;
    if (pitch_rad < limits.pitch_min_rad || pitch_rad > limits.pitch_max_rad)
        return TUNING_OUT_OF_RANGE;
    // Yaw允许跨过整圈边界；传给原视觉协议前折回[-PI, PI]。
    yaw_rad = remainderf(yaw_rad, 2.0f * 3.1415926f);
    raw = (int16_t)(yaw_rad * 32767.0f / 3.1415926f);
    payload[1] = (uint8_t)raw;
    payload[2] = (uint8_t)((uint16_t)raw >> 8);
    // 继续复用视觉八字节载荷和 Pitch 符号，向零取整避免量化越界。
    raw = (int16_t)(-pitch_rad * 32767.0f / 3.1415926f);
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

static uint8_t submit_pitch(float angle_rad)
{
    pitch_tuning_state_t state;
    gimbal_tuning_get_state(&state);
    // 单独修改Pitch时，载荷中的Yaw仍保持原目标。
    return submit_angles(angle_rad, state.yaw.target_angle_rad);
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
    case CMD_YAW:
        if (value < -180.0f || value > 180.0f)
            return TUNING_OUT_OF_RANGE;
        // 单独修改Yaw时保持Pitch目标，避免把它重置到零。
        return submit_angles(state.control.target_angle_rad, value * DEG_TO_RAD);
    case CMD_YAW_WAVE:
        return gimbal_tuning_start_yaw_wave(value, values[1], values[2]);
    case CMD_YAW_PING:
        if (value != 0.0f)
            return TUNING_BAD_PARAMETER;
        return gimbal_tuning_keep_yaw_wave();
    case CMD_YAW_STOP:
        if (value != 0.0f)
            return TUNING_BAD_PARAMETER;
        gimbal_tuning_stop_yaw_wave();
        return TUNING_OK;
    case CMD_STREAM:
        // 十二通道共52字节，5 ms约占90%带宽；30 Hz测试不能仍用50 Hz观察。
        if (value != truncf(value))
            return TUNING_BAD_PARAMETER;
        if (value != 0.0f && (value < 5.0f || value > 2000.0f))
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
    // 格式错误的整行不执行，不把残余内容当作新命令。
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
    const pitch_control_debug_t *control = &state.control;
    const yaw_control_debug_t *yaw = &state.yaw;
    float channels[TUNING_CHANNEL_COUNT];
    uint16_t i;

    gimbal_tuning_get_state(&state);
    channels[0] = control->actual_angle_rad * RAD_TO_DEG;
    channels[1] = control->target_angle_rad * RAD_TO_DEG;
    channels[2] = yaw->actual_angle_rad * RAD_TO_DEG;
    channels[3] = yaw->target_angle_rad * RAD_TO_DEG;
    channels[4] = (float)yaw->motor_cmd;
    channels[5] = (float)state.vision_online;
    channels[6] = yaw->target_speed_rad_s * RAD_TO_DEG;
    channels[7] = yaw->speed_rad_s * RAD_TO_DEG;
    // 直接上传本周期门控记录，单位rad；前置条件提前返回时对应字段为0。
    channels[8] = state.fire_gate.yaw_error_rad;
    channels[9] = state.fire_gate.yaw_threshold_rad;
    channels[10] = state.fire_gate.pitch_error_rad;
    channels[11] = state.fire_gate.pitch_threshold_rad;

    // Inf会被误认作帧尾；仅过滤上传值，不改变控制数据。
    for (i = 0; i < TUNING_CHANNEL_COUNT; ++i)
    {
        if (!isfinite(channels[i]))
        {
            channels[i] = 0.0f;
        }
    }
    return tuning_encode(channels, output);
}

uint16_t tuning_service_period_ms(void)
{
    return stream_period_ms;
}
