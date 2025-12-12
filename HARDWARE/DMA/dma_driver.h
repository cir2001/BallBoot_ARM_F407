#ifndef __DMA_DRIVER_H
#define __DMA_DRIVER_H

#include "stm32f4xx.h"
// ==============================================================================
//  宏定义：缓冲区大小
// ============================================================================== 
#define DMA_TX_BUFFER_SIZE  128  // 发送缓冲区大小
#define DMA_RX_BUFFER_SIZE  256  // 接收缓冲区大小 (建议2的幂次方)

// ==============================================================================
//  外部变量声明 (Extern)
//  让其他文件(main.c)可以直接访问这两个数组
// ============================================================================== */
extern uint8_t dma_tx_buffer[DMA_TX_BUFFER_SIZE];
extern uint8_t dma_rx_buffer[DMA_RX_BUFFER_SIZE];

// ==============================================================================
//  函数声明
// ============================================================================== */
// 初始化 USART1 相关的 DMA (TX: Stream7, RX: Stream2) */
void DMA_Config_USART1(void);
// 初始化 USART2 相关的 DMA (TX: Stream6, RX: Stream2) */
void DMA_Config_USART2(void);
/**
 * @brief  启动 DMA 发送 (非阻塞)
 * @param  len: 要发送的数据长度
 * @return 0: 成功启动, 1: 失败(DMA正忙)
 */
uint8_t DMA_USART1_Start_TX(uint16_t len);

/**
 * @brief  获取当前 RX 缓冲区中已经接收到的数据位置 (用于计算数据长度)
 * @return 当前 DMA 写入位置的索引 (0 ~ RX_BUFFER_SIZE-1)
 */
uint32_t DMA_Get_RX_Current_Pos(void);


void IMU_Send_Data(float gx, float gy, float gz, 
                   float ax, float ay, float az, 
                   float pitch, float roll, float yaw);


#endif /* __DMA_DRIVER_H */


