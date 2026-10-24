#include "tuning_uart.h"
#include "tuning_service.h"
#include "usart.h"
#include <string.h>

#define RX_RING_SIZE 512U
#define PARTIAL_FRAME_TIMEOUT_MS 100U

static uint8_t rx_byte;
static uint8_t rx_ring[RX_RING_SIZE];
static uint8_t rx_gap[RX_RING_SIZE]; // 记录这一个字节之前是否有超过100 ms的空隙
static volatile uint16_t rx_write;
static volatile uint16_t rx_read;
static volatile uint8_t rx_overflow;
static volatile uint8_t rx_restart;
static volatile uint32_t rx_tick;
volatile uint32_t tuning_uart_rx_errors;
volatile uint32_t tuning_uart_overflows;
volatile uint32_t tuning_uart_tx_errors;
static uint8_t tx_bytes[TUNING_FRAME_SIZE];
static uint8_t pending_bytes[TUNING_FRAME_SIZE];
static uint16_t tx_length;
static tuning_parser_t parser;
static tuning_message_t request;
static uint32_t last_stream_tick;

void tuning_uart_init(void)
{
    rx_tick = HAL_GetTick();
    if (HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK)
        rx_restart = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint32_t now;
    uint16_t next;
    if (uart != &huart6)
        return;
    now = HAL_GetTick();
    next = (uint16_t)((rx_write + 1U) % RX_RING_SIZE);
    if (next == rx_read)
    {
        rx_overflow = 1;
        ++tuning_uart_overflows;
    }
    else
    {
        rx_ring[rx_write] = rx_byte;
        rx_gap[rx_write] = now - rx_tick > PARTIAL_FRAME_TIMEOUT_MS;
        __DMB();
        rx_write = next;
    }
    rx_tick = now;
    if (HAL_UART_Receive_IT(uart, &rx_byte, 1) != HAL_OK)
        rx_restart = 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart6)
    {
        ++tuning_uart_rx_errors;
        rx_restart = 1;
    }
}

static void queueTelemetry(uint32_t now)
{
    // 总发送关闭时，命令仍执行，但不生成周期遥测或命令回帧。
    if (TUNING_VOFA_TX_ENABLED == 0)
    {
        return;
    }

    // 待发区独立于DMA区；来不及上传时保留最新快照，不积压过期观测。
    tx_length = tuning_service_frame(pending_bytes);
    last_stream_tick = now;
}

static void sendTelemetry(void)
{
    // 没有待发快照时不启动DMA。
    if (tx_length == 0U)
    {
        return;
    }
    // 发送忙只影响上行；接收和命令执行已经在本轮完成。
    if (huart6.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    memcpy(tx_bytes, pending_bytes, tx_length);
    if (HAL_UART_Transmit_DMA(&huart6, tx_bytes, tx_length) == HAL_OK)
    {
        tx_length = 0;
    }
    else
    {
        // 启动失败保留待发快照，下一轮重试，不阻塞新命令。
        ++tuning_uart_tx_errors;
    }
}

void tuning_uart_poll(void)
{
    uint32_t key, now = HAL_GetTick();
    uint16_t end, period, budget = RX_RING_SIZE;
    uint8_t byte;

    if (rx_restart || rx_overflow)
    {
        /* 丢弃错误前的残包，不把前后两段拼成一条指令。恢复放在任务上下文。 */
        key = __get_PRIMASK();
        __disable_irq();
        rx_read = rx_write;
        rx_overflow = 0;
        tuning_parser_discard(&parser);
        __set_PRIMASK(key);
        if (rx_restart)
        {
            HAL_UART_AbortReceive(&huart6);
            __HAL_UART_CLEAR_OREFLAG(&huart6);
            rx_restart = HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK;
        }
    }

    // 超时后整行作废，不能让后半段碰巧成为有效参数命令。
    if (parser.count != 0U && now - rx_tick > PARTIAL_FRAME_TIMEOUT_MS)
        tuning_parser_discard(&parser);

    end = rx_write;
    while (rx_read != end && budget-- != 0U)
    {
        if (rx_overflow || rx_restart)
            return;
        // 即使后半行已经到达、刷新了全局 rx_tick，也要识别两段之间的超时。
        if (parser.count != 0U && rx_gap[rx_read])
            tuning_parser_discard(&parser);
        byte = rx_ring[rx_read];
        rx_read = (uint16_t)((rx_read + 1U) % RX_RING_SIZE);
        if (tuning_parser_feed(&parser, byte, &request))
        {
            tuning_service_request(&request);
            queueTelemetry(now);
            // 每轮仍只执行一条完整命令，保持原来的处理节奏。
            break;
        }
    }
    period = tuning_service_period_ms();
    // 命令快照优先发送，不被周期遥测替换；没有待发帧时才采集周期快照。
    if (tx_length == 0U)
    {
        if (period != 0U && now - last_stream_tick >= period)
        {
            queueTelemetry(now);
        }
    }
    sendTelemetry();
}
