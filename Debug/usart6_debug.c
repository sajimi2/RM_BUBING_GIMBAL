#include "usart6_debug.h"

extern UART_HandleTypeDef huart6;

// 初始化
void USART6_Debug_Init(void)
{
    // 如果需要可以在这里添加初始化代码
    // USART6已经在CubeMX中初始化，这里通常不需要额外操作
    USART6_SendString("USART6 Debug Initialized\r\n");
    USART6_SendString("System Ready!\r\n");
}

// 基础字符串发送
void USART6_SendString(const char *str)
{
    if(str == NULL) return;
    uint16_t len = strlen(str);
    HAL_UART_Transmit(&huart6, (uint8_t*)str, len, HAL_MAX_DELAY);
}

// 发送原始数据
void USART6_SendData(uint8_t *data, uint16_t len)
{
    if(data == NULL || len == 0) return;
    HAL_UART_Transmit(&huart6, data, len, HAL_MAX_DELAY);
}

// 十六进制格式发送数据
void USART6_SendHex(uint8_t *data, uint16_t len)
{
    char hex_buf[64];  // 增大缓冲区
    char temp[8];
    
    strcpy(hex_buf, "数据: ");
    for(uint16_t i = 0; i < len; i++)
    {
        if(i < len - 1)
            sprintf(temp, "%02X ", data[i]);
        else
            sprintf(temp, "%02X", data[i]);  // 最后一个不加空格
        
        strcat(hex_buf, temp);
    }
    strcat(hex_buf, "\r\n");
    USART6_SendString(hex_buf);
}

// 格式化输出函数
void USART6_Printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    USART6_SendString(buffer);
}

// 重定向 printf（可选配置）
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
