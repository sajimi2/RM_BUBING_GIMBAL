#include "vofa_debug.h"
#include <stddef.h>
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "CAN_recieve_send.h"
#include "NX_communicate.h"
#include "INS_task.h"
#include "gimbal_task.h"
#include "robot_total_mode.h"

#if VOFA_UART_INDEX == 6
#define VOFA_UART huart6
#define VOFA_UART_IRQ USART6_IRQn
#define VOFA_DMA_IRQ DMA2_Stream6_IRQn
#elif VOFA_UART_INDEX == 1
#define VOFA_UART huart1
#define VOFA_UART_IRQ USART1_IRQn
#define VOFA_DMA_IRQ DMA2_Stream7_IRQn
#else
#error VOFA_UART_INDEX must be 1 or 6
#endif

extern volatile uint8_t rc_online;
volatile vofa_debug_stats_t vofa_debug_stats;

typedef struct
{
    float channels[VOFA_CHANNEL_COUNT];
    uint8_t tail[4];
} vofa_frame_t;

/* ARMCC5/GCC 都在编译期检查，确保线上只有 16 个 float 和 4 字节帧尾。 */
typedef char vofa_float_must_be_4_bytes[(sizeof(float) == 4) ? 1 : -1];
typedef char vofa_frame_must_be_68_bytes[(sizeof(vofa_frame_t) == 68) ? 1 : -1];
typedef char vofa_tail_must_follow_channels[(offsetof(vofa_frame_t, tail) == 64) ? 1 : -1];

/* DMA 完成前绝不重写该静态缓冲；串口忙时丢帧，不排队、不等待。 */
static vofa_frame_t frame = { {0}, {0x00, 0x00, 0x80, 0x7F} };
static StaticTask_t task_control_block;
static StackType_t task_stack[256];

typedef struct
{
    int16_t vision_yaw, vision_pitch;
    float ins_yaw, ins_pitch, target_yaw, target_pitch;
    uint16_t yaw_ecd, pitch_ecd;
    int16_t yaw_rpm, pitch_rpm;
    gimbal_rx_observation_t rx;
    uint32_t now_ms;
    uint8_t auto_active, remote_online;
} vofa_snapshot_t;

static float feedback_age(uint32_t now, uint32_t last, uint8_t seen)
{
    /* 从未收到反馈返回 -1；按无符号差计算，兼容 HAL tick 正常回绕。 */
    return seen ? (float)(uint32_t)(now - last) : -1.0f;
}

static void vofa_send_once(void)
{
    vofa_snapshot_t sample;
    const INS_t *ins = get_INS();
    const motor_measure_t *yaw = get_yaw_gimbal_motor_measure_point();
    const motor_measure_t *pitch = get_pitch_gimbal_motor_measure_point();
    const volatile gimbal_rx_observation_t *rx = get_gimbal_rx_observation();

    /* 同时检查 DMA 状态，防止 UART 出错恢复 READY 后仍改写正在发送的内存。 */
    if (VOFA_UART.gState != HAL_UART_STATE_READY ||
        VOFA_UART.hdmatx->State != HAL_DMA_STATE_READY ||
        (VOFA_UART.hdmatx->Instance->CR & DMA_SxCR_EN) != 0)
    {
        vofa_debug_stats.skipped_busy++;
        return;
    }

    /* 仅复制标量。CAN IRQ 为 5/6，短临界区避免半帧；换算和发送均在区外。
     * 不改变 data_ready，不消费视觉帧，不改控制器状态。 */
    taskENTER_CRITICAL();
    uint32_t begin_cycles = DWT->CYCCNT;
    sample.vision_yaw = NX_data.yaw;
    sample.vision_pitch = NX_data.pitch;
    sample.ins_yaw = ins->Yaw;
    sample.ins_pitch = ins->Pitch;
    sample.target_yaw = get_yaw_target_angle();
    sample.target_pitch = get_pitch_target_angle();
    sample.yaw_ecd = yaw->ecd;
    sample.pitch_ecd = pitch->ecd;
    sample.yaw_rpm = yaw->speed_rpm;
    sample.pitch_rpm = pitch->speed_rpm;
    sample.rx = *rx;
    sample.now_ms = HAL_GetTick();
    sample.remote_online = rc_online;
    sample.auto_active = (uint8_t)(robotIsAuto() && *getRobotPresentMode() != RobotState_e_Powerless);
    uint32_t snapshot_cycles = DWT->CYCCNT - begin_cycles;
    taskEXIT_CRITICAL();
    if (snapshot_cycles > vofa_debug_stats.snapshot_max_cycles)
        vofa_debug_stats.snapshot_max_cycles = snapshot_cycles;

    /* 通道 0～5 统一为 rad，与控制内部单位一致。
     * 视觉 CAN 定点值按原协议还原，保留发送端符号；Pitch 控制目标已取反并限幅。 */
    frame.channels[0] = (float)sample.vision_yaw * 3.1415926f / 32767.0f;
    frame.channels[1] = (float)sample.vision_pitch * 3.1415926f / 32767.0f;
    frame.channels[2] = sample.ins_yaw;
    frame.channels[3] = sample.ins_pitch;
    frame.channels[4] = sample.target_yaw;
    frame.channels[5] = sample.target_pitch;
    frame.channels[6] = (float)sample.yaw_ecd * (360.0f / 8192.0f);
    frame.channels[7] = (float)sample.pitch_ecd * (360.0f / 8192.0f);
    frame.channels[8] = (float)sample.yaw_rpm;
    frame.channels[9] = (float)sample.pitch_rpm;
    frame.channels[10] = feedback_age(sample.now_ms, sample.rx.vision_last_ms, sample.rx.vision_seen);
    frame.channels[11] = feedback_age(sample.now_ms, sample.rx.yaw_last_ms, sample.rx.yaw_seen);
    frame.channels[12] = feedback_age(sample.now_ms, sample.rx.pitch_last_ms, sample.rx.pitch_seen);
    frame.channels[13] = (float)sample.auto_active;
    frame.channels[14] = (float)sample.remote_online;
    frame.channels[15] = (float)sample.now_ms * 0.001f;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&VOFA_UART, (const uint8_t *)&frame, sizeof(frame));
    if (status == HAL_OK)
    {
        /* 无半帧处理需求；保留完成/错误 IRQ 和 UART TC IRQ。 */
        __HAL_DMA_DISABLE_IT(VOFA_UART.hdmatx, DMA_IT_HT);
        vofa_debug_stats.submitted++;
    }
    else if (status == HAL_BUSY)
        vofa_debug_stats.skipped_busy++;
    else
        vofa_debug_stats.start_errors++;
}

static void vofa_task(void *argument)
{
    (void)argument;
    for (;;)
    {
        /* 相对延时不补发积压采样；高优先级控制繁忙时允许遥测降频。 */
        vTaskDelay(pdMS_TO_TICKS(VOFA_PERIOD_MS));
        vofa_send_once();
    }
}

void VOFA_Debug_Start(void)
{
    /* 遥测 IRQ 低于 CAN(5/6)，任务低于已有普通/控制/IMU任务。
     * 栈和 TCB 静态分配，不挤占原有 FreeRTOS 24 KB 动态堆。 */
    HAL_NVIC_SetPriority(VOFA_UART_IRQ, 7, 0);
    HAL_NVIC_SetPriority(VOFA_DMA_IRQ, 7, 0);
    (void)xTaskCreateStatic(vofa_task, "vofa", 256, NULL, tskIDLE_PRIORITY + 1,
                            task_stack, &task_control_block);
}
