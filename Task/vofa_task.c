/*
 * @file vofa_task.c
 * @brief VOFA任务
 * @author Zhao Zhenghao
 * @date 2024-06-20
 *
 * 功能定义：VOFA的debug任务，用来观察曲线
 */

#include "vofa_task.h"
#include "tuning_uart.h"
#include "cmsis_os.h"

#define VOFA_POLL_PERIOD_MS 5U

void vofa_task(void const *argument)
{
    (void)argument;

    // 启动USART6接收；后续命令解析、状态打包和DMA发送由本任务独占处理。
    tuning_uart_init();

    while (1)
    {
        tuning_uart_poll();

        // 保持原来的5 ms轮询节奏；实际遥测周期仍由stream参数决定。
        osDelay(VOFA_POLL_PERIOD_MS);
    }
}
