#include "bsp_usart.h"
#include "NX_communicate.h"
#include "Robot_Information.h"
#include "usart.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "robot_total_mode.h"
#include <string.h>

extern DMA_HandleTypeDef hdma_usart1_rx;

VisionToGimbal_t NX_data;
volatile uint8_t data_ready = 0;

#define RX_DMA_BUFFER_SIZE 256

// 双缓冲区
static uint8_t rx_dma_buf1[RX_DMA_BUFFER_SIZE];
static uint8_t rx_dma_buf2[RX_DMA_BUFFER_SIZE];

// CRC16 Table
static const uint16_t CRC16_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

static uint16_t get_crc16(const uint8_t * data, uint32_t len)
{
    uint16_t crc16 = 0xffff;
    uint8_t byte;
    uint8_t i;

    while (len--) {
        byte = *data++;
        i = (crc16 ^ byte) & 0x00ff;
        crc16 = (crc16 >> 8) ^ CRC16_TABLE[i];
    }
    return crc16;
}

static uint8_t check_crc16(const uint8_t * data, uint32_t len)
{
    uint16_t crc16 = (data[len - 1] << 8) | data[len - 2];
    return get_crc16(data, len - 2) == crc16;
}

void NXrecieve_Init(void)
{
    // 启动双缓冲 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf1, RX_DMA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx_dma_buf1);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx_dma_buf2);
}

static void NX_ProcessBuffer(uint8_t *buf, uint16_t size)
{
    // 最小包长检查
    if(size < sizeof(VisionToGimbal_t)) return;

    // 遍历 buffer 查找包头 'S', 'P'
    for(uint16_t i = 0; i + sizeof(VisionToGimbal_t) <= size; i++)
    {
        if(buf[i] == 'S' && buf[i+1] == 'P')
        {
            if(check_crc16(&buf[i], sizeof(VisionToGimbal_t)))
            {
                memcpy(&NX_data, &buf[i], sizeof(VisionToGimbal_t));
                data_ready = 1;
                // 找到一个包后跳过该包继续查找
                i += sizeof(VisionToGimbal_t) - 1; 
            }
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance != USART1) 
			return;

    uint8_t *active_buf;
    uint8_t *next_buf;

    // 获取当前接收完成的 Buffer 地址
    active_buf = huart->pRxBuffPtr;
    
    // 决定下一个使用的 Buffer（软双缓冲切换）
    if (active_buf == rx_dma_buf1)
    {
        next_buf = rx_dma_buf2;
    }
    else
    {
        next_buf = rx_dma_buf1;
    }

    // 处理当前接收到的数据
    NX_ProcessBuffer(active_buf, Size);

    // 重新启动 DMA 接收，使用下一个 Buffer
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, next_buf, RX_DMA_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

// 添加错误回调以处理 Overrun Error (ORE)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        // 如果发生 Overrun 错误，清除标志并重启接收
        if(HAL_IS_BIT_SET(huart->Instance->SR, UART_FLAG_ORE))
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
            // 重启接收，复位到 buf1
            HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_dma_buf1, RX_DMA_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    }
}

void Send_Gimbal_Information(void)
{
    static GimbalToVision_t tx_data;
    
    tx_data.head[0] = 'S';
    tx_data.head[1] = 'P';
    
    // Mode
    if(robotIsAuto())
        tx_data.mode = 1; // Auto Aim
    else
        tx_data.mode = 0; // Idle
        
    // Quaternion
    extern INS_t INS; // INS is global in INS_task.c
    tx_data.q[0] = INS.q[0];
    tx_data.q[1] = INS.q[1];
    tx_data.q[2] = INS.q[2];
    tx_data.q[3] = INS.q[3];
    
    // Gimbal Info
    tx_data.yaw = get_yaw_current_angle();
    tx_data.yaw_vel = get_yaw_current_speed();
    tx_data.pitch = get_pitch_current_angle();
    tx_data.pitch_vel = get_pitch_current_speed();
    
    // Bullet Info (Data source currently unavailable in project)
    tx_data.bullet_speed = 20.0f;
    tx_data.bullet_count = 0;
    
    // CRC
    tx_data.crc16 = get_crc16((uint8_t*)&tx_data, sizeof(tx_data) - 2);
    
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&tx_data, sizeof(tx_data));
}

void NX_task(void const *pvParameters)
{
    osDelay(100);
    while(1)
    {
        Send_Gimbal_Information();
        osDelay(100); // 100Hz
    }
}

const VisionToGimbal_t *get_nx_control_point(void)
{
    return &NX_data;
}
