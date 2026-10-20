#ifndef TUNING_UART_H
#define TUNING_UART_H

/* 由现有低优先级 test_task 调用，不在云台两毫秒控制循环里解析或发送。 */
void tuning_uart_init(void);
void tuning_uart_poll(void);

#endif
