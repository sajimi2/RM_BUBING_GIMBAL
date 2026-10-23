#ifndef TUNING_UART_H
#define TUNING_UART_H

/* 仅由低优先级vofa_task调用，不在云台两毫秒控制循环里解析或发送。 */
void tuning_uart_init(void);
void tuning_uart_poll(void);

#endif
