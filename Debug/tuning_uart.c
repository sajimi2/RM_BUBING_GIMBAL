#include "tuning_uart.h"
#include "tuning_service.h"
#include "usart.h"

#define RX_RING_SIZE 512U
#define PARTIAL_FRAME_TIMEOUT_MS 100U

static uint8_t rx_byte;
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_write;
static volatile uint16_t rx_read;
static volatile uint8_t rx_overflow;
static volatile uint8_t rx_restart;
static volatile uint32_t rx_tick;
volatile uint32_t tuning_uart_rx_errors;
volatile uint32_t tuning_uart_overflows;
volatile uint32_t tuning_uart_tx_errors;
static uint8_t tx_bytes[TUNING_FRAME_SIZE];
static uint16_t tx_length;
static tuning_parser_t parser;
static tuning_message_t request;
static tuning_message_t reply;
static uint32_t last_stream_tick;

void tuning_uart_init(void)
{
    rx_tick = HAL_GetTick();
    if (HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK)
        rx_restart = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint16_t next;
    if (uart != &huart6)
        return;
    next = (uint16_t)((rx_write + 1U) % RX_RING_SIZE);
    if (next == rx_read)
    {
        rx_overflow = 1;
        ++tuning_uart_overflows;
    }
    else
    {
        rx_ring[rx_write] = rx_byte;
        __DMB();
        rx_write = next;
    }
    rx_tick = HAL_GetTick();
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
        tuning_parser_reset(&parser);
        __set_PRIMASK(key);
        if (rx_restart)
        {
            HAL_UART_AbortReceive(&huart6);
            __HAL_UART_CLEAR_OREFLAG(&huart6);
            rx_restart = HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK;
        }
    }

    /* DMA 持有 tx_bytes 期间不能改写。回复也不能被遥测覆盖。 */
    if (huart6.gState != HAL_UART_STATE_READY)
        return;
    if (tx_length != 0U)
    {
        if (HAL_UART_Transmit_DMA(&huart6, tx_bytes, tx_length) == HAL_OK)
            tx_length = 0;
        else
            ++tuning_uart_tx_errors;
        return;
    }
    if ((int32_t)(now - rx_tick) > (int32_t)PARTIAL_FRAME_TIMEOUT_MS)
        tuning_parser_reset(&parser);

    end = rx_write;
    while (rx_read != end && budget-- != 0U)
    {
        if (rx_overflow || rx_restart)
            return;
        byte = rx_ring[rx_read];
        rx_read = (uint16_t)((rx_read + 1U) % RX_RING_SIZE);
        if (tuning_parser_feed(&parser, byte, &request))
        {
            if ((request.command & 0x80U) || request.command == TUNING_TELEMETRY)
                continue;
            tuning_service_request(&request, &reply);
            tx_length = tuning_encode(&reply, tx_bytes);
            return;
        }
    }
    period = tuning_service_period_ms();
    if (period != 0U && now - last_stream_tick >= period)
    {
        tuning_service_state(&reply);
        reply.command = TUNING_TELEMETRY;
        reply.sequence = 0;
        tx_length = tuning_encode(&reply, tx_bytes);
        last_stream_tick = now;
    }
}
