#ifndef __USART6_DEBUG_H
#define __USART6_DEBUG_H

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"

// 重定向 printf 到 USART6（可选）
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

// 初始化函数
void USART6_Debug_Init(void);

// 基础发送函数
void USART6_SendString(const char *str);
void USART6_SendData(uint8_t *data, uint16_t len);
void USART6_SendHex(uint8_t *data, uint16_t len);

// 格式化发送函数
void USART6_Printf(const char *format, ...);

// 调试信息宏
#define DEBUG_INFO(...) USART6_Printf("[INFO] " __VA_ARGS__)
#define DEBUG_WARN(...) USART6_Printf("[WARN] " __VA_ARGS__)
#define DEBUG_ERROR(...) USART6_Printf("[ERROR] " __VA_ARGS__)

#endif /* __USART6_DEBUG_H */
